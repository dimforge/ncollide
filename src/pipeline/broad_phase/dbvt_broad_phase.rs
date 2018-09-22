use bounding_volume::{BoundingVolume, BoundingVolumeInterferencesCollector};
use math::Point;
use na::Real;
use partitioning::{DBVT, DBVTLeaf, DBVTLeafId};
use pipeline::broad_phase::{BroadPhase, BroadPhaseInterferenceHandler, ProxyHandle};
use query::{PointInterferencesCollector, PointQuery, Ray, RayCast, RayInterferencesCollector};
use slab::Slab;
use std::any::Any;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::mem;
use utils::{DeterministicState, SortedPair};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum ProxyStatus {
    OnStaticTree(DBVTLeafId),
    OnDynamicTree(DBVTLeafId, usize),
    // The usize is the location of the corresponding on proxies_to_update
    Detached(Option<usize>),
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
pub struct DBVTBroadPhase<N: Real, BV, T> {
    proxies: Slab<DBVTBroadPhaseProxy<T>>,
    // DBVT for moving objects.
    tree: DBVT<N, ProxyHandle, BV>,
    // DBVT for static objects.
    stree: DBVT<N, ProxyHandle, BV>,
    // Pairs detected.
    pairs: HashMap<SortedPair<ProxyHandle>, bool, DeterministicState>,
    // The margin added to each bounding volume.
    margin: N,
    purge_all: bool,

    // Just to avoid dynamic allocations.
    collector: Vec<ProxyHandle>,
    leaves_to_update: Vec<DBVTLeaf<N, ProxyHandle, BV>>,
    proxies_to_update: Vec<(ProxyHandle, BV)>,
}

impl<N, BV, T> DBVTBroadPhase<N, BV, T>
    where
        N: Real,
        BV: 'static + BoundingVolume<N> + Clone,
{
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(margin: N) -> DBVTBroadPhase<N, BV, T> {
        DBVTBroadPhase {
            proxies: Slab::new(),
            tree: DBVT::new(),
            stree: DBVT::new(),
            pairs: HashMap::with_hasher(DeterministicState::new()),
            purge_all: false,
            collector: Vec::new(),
            leaves_to_update: Vec::new(),
            proxies_to_update: Vec::new(),
            margin: margin,
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> usize {
        self.pairs.len()
    }

    fn purge_some_contact_pairs(&mut self, handler: &mut BroadPhaseInterferenceHandler<T>) {
        let purge_all = self.purge_all;
        let proxies = &self.proxies;
        let stree = &self.stree;
        let tree = &self.tree;
        self.pairs.retain(|pair, up_to_date| {
            let mut retain = true;

            if purge_all || !*up_to_date {
                *up_to_date = true;

                let proxy1 = proxies
                    .get(pair.0.uid())
                    .expect("DBVT broad phase: internal error.");
                let proxy2 = proxies
                    .get(pair.1.uid())
                    .expect("DBVT broad phase: internal error.");

                if purge_all || proxy1.updated || proxy2.updated {
                    if handler.is_interference_allowed(&proxy1.data, &proxy2.data) {
                        let l1 = match proxy1.status {
                            ProxyStatus::OnStaticTree(leaf) => &stree[leaf],
                            ProxyStatus::OnDynamicTree(leaf, _) => &tree[leaf],
                            _ => panic!("DBVT broad phase: internal error."),
                        };

                        let l2 = match proxy2.status {
                            ProxyStatus::OnStaticTree(leaf) => &stree[leaf],
                            ProxyStatus::OnDynamicTree(leaf, _) => &tree[leaf],
                            _ => panic!("DBVT broad phase: internal error."),
                        };

                        if !l1.bounding_volume.intersects(&l2.bounding_volume) {
                            handler.interference_stopped(&proxy1.data, &proxy2.data);
                            retain = false;
                        }
                    }
                }
            }

            *up_to_date = false;
            retain
        });
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

impl<N, BV, T> BroadPhase<N, BV, T> for DBVTBroadPhase<N, BV, T>
    where
        N: Real,
        BV: BoundingVolume<N> + RayCast<N> + PointQuery<N> + Any + Send + Sync + Clone,
        T: Any + Send + Sync,
{
    fn update(&mut self, handler: &mut BroadPhaseInterferenceHandler<T>) {
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
        let some_leaves_updated = self.leaves_to_update.len() != 0;
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

                    if handler.is_interference_allowed(&proxy1.data, &proxy2.data) {
                        match self.pairs.entry(SortedPair::new(leaf.data, *proxy_key2)) {
                            Entry::Occupied(entry) => *entry.into_mut() = true,
                            Entry::Vacant(entry) => {
                                handler.interference_started(&proxy1.data, &proxy2.data);
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

        if some_leaves_updated {
            self.purge_some_contact_pairs(handler);
        }
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

        {
            let proxies = &self.proxies;
            self.pairs.retain(|pair, _| {
                let proxy1 = proxies
                    .get(pair.0.uid())
                    .expect("DBVT broad phase: internal error.");
                let proxy2 = proxies
                    .get(pair.1.uid())
                    .expect("DBVT broad phase: internal error.");

                if proxy1.status == ProxyStatus::Deleted || proxy2.status == ProxyStatus::Deleted {
                    handler(&proxy1.data, &proxy2.data);
                    false
                } else {
                    true
                }
            });
        }

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

    fn deferred_recompute_all_proximities_with(&mut self, handle: ProxyHandle) {
        if let Some(proxy) = self.proxies.get(handle.uid()) {
            let bv = match proxy.status {
                ProxyStatus::OnStaticTree(leaf) => self.stree[leaf].bounding_volume.clone(),
                ProxyStatus::OnDynamicTree(leaf, _) => self.tree[leaf].bounding_volume.clone(),
                ProxyStatus::Detached(_) => return,
                ProxyStatus::Deleted => {
                    panic!("DBVT broad phase: internal error, proxy not found.")
                }
            };

            self.proxies_to_update.push((handle, bv));
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

    fn interferences_with_ray<'a>(&'a self, ray: &Ray<N>, out: &mut Vec<&'a T>) {
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

    fn interferences_with_point<'a>(&'a self, point: &Point<N>, out: &mut Vec<&'a T>) {
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
