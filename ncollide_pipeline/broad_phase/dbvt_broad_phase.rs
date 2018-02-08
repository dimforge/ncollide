use std::any::Any;
use std::mem;
use std::collections::HashMap;
use std::collections::hash_map::Entry;
use slab::Slab;

use alga::general::Id;
use math::Point;
use utils::data::SortedPair;
use geometry::bounding_volume::{BoundingVolume, BoundingVolumeInterferencesCollector};
use geometry::partitioning::{DBVTLeaf, DBVTLeafId, DBVT};
use geometry::query::{PointInterferencesCollector, PointQuery, Ray, RayCast,
                      RayInterferencesCollector};
use broad_phase::{BroadPhase, ProxyHandle};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ProxyStatus {
    OnStaticTree(DBVTLeafId),
    OnDynamicTree(DBVTLeafId, usize),
    Detached(Option<usize>), // The usize is the location of the corresponding on proxies_to_update
    Deleted,
}

struct DBVTBroadPhaseProxy<T> {
    data: T,
    status: ProxyStatus,
    updated: bool,
}

impl<T> DBVTBroadPhaseProxy<T> {
    fn new(data: T) -> DBVTBroadPhaseProxy<T> {
        DBVTBroadPhaseProxy {
            data: data,
            status: ProxyStatus::Detached(None),
            updated: true,
        }
    }

    fn is_detached(&self) -> bool {
        match self.status {
            ProxyStatus::Detached(_) => true,
            _ => false,
        }
    }
}

const DEACTIVATION_THRESHOLD: usize = 100;

/// Broad phase based on a Dynamic Bounding Volume Tree.
///
/// It uses two separate trees: one for static objects and which is never updated, and one for
/// moving objects.
pub struct DBVTBroadPhase<P: Point, BV, T> {
    proxies: Slab<DBVTBroadPhaseProxy<T>>,
    tree: DBVT<P, ProxyHandle, BV>,  // DBVT for moving objects.
    stree: DBVT<P, ProxyHandle, BV>, // DBVT for static objects.
    pairs: HashMap<SortedPair<ProxyHandle>, bool>, // Pairs detected.
    margin: P::Real,                 // The margin added to each bounding volume.
    purge_all: bool,

    // Just to avoid dynamic allocations.
    collector: Vec<ProxyHandle>,
    pairs_to_remove: Vec<SortedPair<ProxyHandle>>,
    leaves_to_update: Vec<DBVTLeaf<P, ProxyHandle, BV>>,
    proxies_to_update: Vec<(ProxyHandle, BV)>,
}

impl<P, BV, T> DBVTBroadPhase<P, BV, T>
where
    P: Point,
    BV: 'static + BoundingVolume<P> + Clone,
{
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(margin: P::Real) -> DBVTBroadPhase<P, BV, T> {
        DBVTBroadPhase {
            proxies: Slab::new(),
            tree: DBVT::new(),
            stree: DBVT::new(),
            pairs: HashMap::new(),
            purge_all: false,
            collector: Vec::new(),
            leaves_to_update: Vec::new(),
            pairs_to_remove: Vec::new(),
            proxies_to_update: Vec::new(),
            margin: margin,
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> usize {
        self.pairs.len()
    }

    fn purge_some_contact_pairs(
        &mut self,
        allow_proximity: &mut FnMut(&T, &T) -> bool,
        handler: &mut FnMut(&T, &T, bool),
    ) {
        // NOTE: the exact same code is used on `brute_force_bounding_volume_broad_phase.rs`.
        // Refactor that?
        if self.purge_all || (self.leaves_to_update.len() != 0 && self.pairs.len() != 0) {
            for (pair, up_to_date) in &mut self.pairs {
                if self.purge_all || !*up_to_date {
                    *up_to_date = true;
                    let mut remove = true;

                    let proxy1 = self.proxies
                        .get(pair.0.uid())
                        .expect("DBVT broad phase: internal error.");
                    let proxy2 = self.proxies
                        .get(pair.1.uid())
                        .expect("DBVT broad phase: internal error.");

                    if self.purge_all || proxy1.updated || proxy2.updated {
                        if allow_proximity(&proxy1.data, &proxy2.data) {
                            let l1 = match proxy1.status {
                                ProxyStatus::OnStaticTree(leaf) => &self.stree[leaf],
                                ProxyStatus::OnDynamicTree(leaf, _) => &self.tree[leaf],
                                _ => panic!("DBVT broad phase: internal error."),
                            };

                            let l2 = match proxy2.status {
                                ProxyStatus::OnStaticTree(leaf) => &self.stree[leaf],
                                ProxyStatus::OnDynamicTree(leaf, _) => &self.tree[leaf],
                                _ => panic!("DBVT broad phase: internal error."),
                            };

                            if l1.bounding_volume.intersects(&l2.bounding_volume) {
                                remove = false
                            }
                        }
                    } else {
                        remove = false;
                    }

                    if remove {
                        handler(&proxy1.data, &proxy2.data, false);
                        self.pairs_to_remove.push(*pair);
                    }
                }
            }
        }

        /*
         * Actually remove the pairs.
         */
        for pair in self.pairs_to_remove.iter() {
            let _ = self.pairs.remove(pair);
        }

        self.pairs_to_remove.clear();
    }

    fn update_activation_states(&mut self) {
        /*
         * Update activation states.
         * FIXME: could we avoid having to iterate through _all_ the proxies at each update?
         */
        for (_, proxy) in self.proxies.iter_mut() {
            if let ProxyStatus::OnDynamicTree(leaf, energy) = proxy.status {
                if energy == 1 {
                    let old_leaf = self.tree.remove(leaf);
                    let new_leaf = self.stree.insert(old_leaf);
                    proxy.status = ProxyStatus::OnStaticTree(new_leaf);
                } else {
                    proxy.status = ProxyStatus::OnDynamicTree(leaf, energy - 1)
                }
            }
        }
    }
}

impl<P, BV, T> BroadPhase<P, BV, T> for DBVTBroadPhase<P, BV, T>
where
    P: Point,
    BV: BoundingVolume<P> + RayCast<P, Id> + PointQuery<P, Id> + Any + Send + Sync + Clone,
    T: Any + Send + Sync,
{
    fn update(
        &mut self,
        allow_proximity: &mut FnMut(&T, &T) -> bool,
        handler: &mut FnMut(&T, &T, bool),
    ) {
        /*
         * Remove from the trees all nodes that have been deleted or modified.
         */
        for (handle, bv) in self.proxies_to_update.drain(..) {
            if let Some(proxy) = self.proxies.get_mut(handle.uid()) {
                let mut set_status = true;
                match proxy.status {
                    ProxyStatus::OnStaticTree(leaf) => {
                        let mut leaf = self.stree.remove(leaf);
                        leaf.bounding_volume = bv;
                        self.leaves_to_update.push(leaf);
                    }
                    ProxyStatus::OnDynamicTree(leaf, _) => {
                        let mut leaf = self.tree.remove(leaf);
                        leaf.bounding_volume = bv;
                        self.leaves_to_update.push(leaf);
                    }
                    ProxyStatus::Detached(None) => {
                        let leaf = DBVTLeaf::new(bv, handle);
                        self.leaves_to_update.push(leaf);
                    }
                    ProxyStatus::Detached(Some(id)) => {
                        let leaf = DBVTLeaf::new(bv, handle);
                        self.leaves_to_update[id] = leaf;
                        set_status = false;
                    }
                    ProxyStatus::Deleted => {
                        panic!("DBVT broad phase internal error: the proxy was deleted.")
                    }
                }

                proxy.updated = true;

                if set_status {
                    proxy.status = ProxyStatus::Detached(Some(self.leaves_to_update.len() - 1));
                }
            }
        }

        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        for leaf in self.leaves_to_update.drain(..) {
            {
                let proxy1 = &self.proxies[leaf.data.uid()];
                {
                    let mut visitor = BoundingVolumeInterferencesCollector::new(
                        &leaf.bounding_volume,
                        &mut self.collector,
                    );

                    self.tree.visit(&mut visitor);
                    self.stree.visit(&mut visitor);
                }

                // Event generation.
                for proxy_key2 in self.collector.iter() {
                    let proxy2 = &self.proxies[proxy_key2.uid()];

                    if allow_proximity(&proxy1.data, &proxy2.data) {
                        match self.pairs.entry(SortedPair::new(leaf.data, *proxy_key2)) {
                            Entry::Occupied(entry) => *entry.into_mut() = true,
                            Entry::Vacant(entry) => {
                                handler(&proxy1.data, &proxy2.data, true);
                                let _ = entry.insert(true);
                            }
                        }
                    }
                }

                self.collector.clear();
            }

            let proxy1 = &mut self.proxies[leaf.data.uid()];
            assert!(proxy1.is_detached());
            let leaf = self.tree.insert(leaf);
            proxy1.status = ProxyStatus::OnDynamicTree(leaf, DEACTIVATION_THRESHOLD);
        }

        self.purge_some_contact_pairs(allow_proximity, handler);
        self.update_activation_states();
    }

    fn create_proxy(&mut self, bv: BV, data: T) -> ProxyHandle {
        let proxy = DBVTBroadPhaseProxy::new(data);
        let handle = ProxyHandle(self.proxies.insert(proxy));
        self.proxies_to_update.push((handle, bv));
        handle
    }

    fn remove(&mut self, handles: &[ProxyHandle], handler: &mut FnMut(&T, &T)) {
        for handle in handles {
            if let Some(proxy) = self.proxies.get_mut(handle.uid()) {
                match proxy.status {
                    ProxyStatus::OnStaticTree(leaf) => {
                        let _ = self.stree.remove(leaf);
                    }
                    ProxyStatus::OnDynamicTree(leaf, _) => {
                        let _ = self.tree.remove(leaf);
                    }
                    _ => {}
                }

                proxy.status = ProxyStatus::Deleted;
            } else {
                panic!("Attempting to remove an object that does not exist.");
            }
        }

        for (pair, _) in &mut self.pairs {
            let proxy1 = self.proxies
                .get(pair.0.uid())
                .expect("DBVT broad phase: internal error.");
            let proxy2 = self.proxies
                .get(pair.1.uid())
                .expect("DBVT broad phase: internal error.");

            if proxy1.status == ProxyStatus::Deleted || proxy2.status == ProxyStatus::Deleted {
                handler(&proxy1.data, &proxy2.data);
                self.pairs_to_remove.push(*pair)
            }
        }

        for pair in self.pairs_to_remove.iter() {
            let _ = self.pairs.remove(pair);
        }
        self.pairs_to_remove.clear();

        for handle in handles {
            let _ = self.proxies.remove(handle.uid());
        }
    }

    fn deferred_set_bounding_volume(&mut self, handle: ProxyHandle, bounding_volume: BV) {
        if let Some(proxy) = self.proxies.get(handle.uid()) {
            let needs_update = match proxy.status {
                ProxyStatus::OnStaticTree(leaf) => {
                    !self.stree[leaf].bounding_volume.contains(&bounding_volume)
                }
                ProxyStatus::OnDynamicTree(leaf, _) => {
                    !self.tree[leaf].bounding_volume.contains(&bounding_volume)
                }
                ProxyStatus::Detached(_) => true,
                ProxyStatus::Deleted => {
                    panic!("DBVT broad phase: internal error, proxy not found.")
                }
            };

            if needs_update {
                let new_bv = bounding_volume.loosened(self.margin);
                self.proxies_to_update.push((handle, new_bv));
            }
        } else {
            panic!("Attempting to set the bounding volume of an object that does not exist.");
        }
    }

    fn deferred_recompute_all_proximities(&mut self) {
        let mut user_updates = mem::replace(&mut self.proxies_to_update, Vec::new());

        for (handle, proxy) in self.proxies.iter() {
            let bv;
            match proxy.status {
                ProxyStatus::OnStaticTree(leaf) => {
                    bv = self.stree[leaf].bounding_volume.clone();
                }
                ProxyStatus::OnDynamicTree(leaf, _) => {
                    bv = self.tree[leaf].bounding_volume.clone();
                }
                ProxyStatus::Detached(_) => continue,
                ProxyStatus::Deleted => {
                    panic!("DBVT broad phase: internal error, proxy not found.")
                }
            }

            self.proxies_to_update.push((ProxyHandle(handle), bv));
        }

        self.proxies_to_update.append(&mut user_updates);
        self.purge_all = true;
    }

    fn interferences_with_bounding_volume<'a>(&'a self, bv: &BV, out: &mut Vec<&'a T>) {
        let mut collector = Vec::new();

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(bv, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l.uid()].data)
        }
    }

    fn interferences_with_ray<'a>(&'a self, ray: &Ray<P>, out: &mut Vec<&'a T>) {
        let mut collector = Vec::new();

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l.uid()].data)
        }
    }

    fn interferences_with_point<'a>(&'a self, point: &P, out: &mut Vec<&'a T>) {
        let mut collector = Vec::new();

        {
            let mut visitor = PointInterferencesCollector::new(point, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l.uid()].data)
        }
    }
}
