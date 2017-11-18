use std::any::Any;
use std::collections::hash_map::{Entry, HashMap as StdHashMap};

use alga::general::Id;
use na;
use utils::data::uid_remap::{UidRemap, FastKey};
use utils::data::pair::{Pair, PairTWHash};
use utils::data::hash_map::HashMap;
use math::Point;
use geometry::bounding_volume::{BoundingVolume, BoundingVolumeInterferencesCollector};
use geometry::partitioning::{DBVT2, DBVTLeaf2, DBVTLeafId};
use geometry::query::{Ray, RayCast, RayInterferencesCollector, PointQuery, PointInterferencesCollector};
use broad_phase::BroadPhase;

#[derive(Copy, Clone, Debug)]
enum ProxyStatus {
    OnStaticTree,
    OnDynamicTree,
    Detached
}

enum Action<BV, T> {
    Modify(BV, Option<T>),
    Remove(usize)
}

struct DBVTBroadPhaseProxy<T> {
    data:   T,
    status: usize,
    leaf:   DBVTLeafId
}

impl<T> DBVTBroadPhaseProxy<T> {
    fn new(data: T) -> DBVTBroadPhaseProxy<T> {
        DBVTBroadPhaseProxy {
            data:   data,
            status: DEACTIVATION_THRESHOLD,
            leaf:   DBVTLeafId::new_invalid()
        }
    }

    fn status(&self) -> ProxyStatus {
        if self.leaf.is_invalid() {
            ProxyStatus::Detached
        }
        else if self.status == 0 {
            ProxyStatus::OnStaticTree
        }
        else {
            ProxyStatus::OnDynamicTree
        }
    }

    fn detach(&mut self) {
        self.leaf   = DBVTLeafId::new_invalid();
        self.status = DEACTIVATION_THRESHOLD;
    }
}

const DEACTIVATION_THRESHOLD: usize = 100;

/// Broad phase based on a Dynamic Bounding Volume Tree.
///
/// It uses two separate trees: one for static objects and which is never updated, and one for
/// moving objects.
pub struct DBVTBroadPhase<P: Point, BV, T> {
    proxies:       UidRemap<DBVTBroadPhaseProxy<T>>,
    rem_proxies:   UidRemap<DBVTBroadPhaseProxy<T>>, // Removed proxies
    tree:          DBVT2<P, FastKey, BV>, // DBVT for moving objects.
    stree:         DBVT2<P, FastKey, BV>, // DBVT for static objects.
    pairs:         HashMap<Pair, (), PairTWHash>, // Pairs detected (FIXME: use a Vec instead?)
    margin:        P::Real, // The margin added to each bounding volume.
    update_off:    usize, // Incremental pairs removal index.
    purge_all:     bool,

    // Just to avoid dynamic allocations.
    collector:         Vec<FastKey>,
    pairs_to_remove:   Vec<Pair>,
    to_update:         Vec<DBVTLeaf2<P, FastKey, BV>>,
    actions:           StdHashMap<FastKey, Action<BV, T>>
}

impl<P, BV, T> DBVTBroadPhase<P, BV, T>
    where P:  Point,
          BV: 'static + BoundingVolume<P> + Clone {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(margin: P::Real, small_keys: bool) -> DBVTBroadPhase<P, BV, T> {
        DBVTBroadPhase {
            proxies:           UidRemap::new(small_keys),
            rem_proxies:       UidRemap::new(small_keys),
            tree:              DBVT2::new(),
            stree:             DBVT2::new(),
            pairs:             HashMap::new(PairTWHash::new()),
            update_off:        0,
            purge_all:         false,
            collector:         Vec::new(),
            to_update:         Vec::new(),
            pairs_to_remove:   Vec::new(),
            actions:           StdHashMap::new(),
            margin:            margin
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> usize {
        self.pairs.len()
    }

    fn purge_some_contact_pairs(&mut self,
                                allow_proximity: &mut FnMut(&T, &T) -> bool,
                                handler:         &mut FnMut(&T, &T, bool)) {
        // NOTE: the exact same code is used on `brute_force_bounding_volume_broad_phase.rs`.
        // Refactor that?
        if self.purge_all || (self.to_update.len() != 0 && self.pairs.len() != 0) {
            let len = self.pairs.len();
            let num_removals;

            if self.purge_all {
                self.update_off = 0;
                num_removals = len;
                self.purge_all = false;
            }
            else {
                num_removals = *na::clamp(&(self.to_update.len()), &(len / 10), &len);
            }

            for i in self.update_off .. self.update_off + num_removals {
                let id   = i % self.pairs.len();
                let elts = self.pairs.elements();
                let ids  = elts[id].key;

                let mut remove = true;

                let proxy1 = self.proxies.get_fast(&ids.first);
                let proxy2 = self.proxies.get_fast(&ids.second);
                if let (Some(proxy1), Some(proxy2)) = (proxy1, proxy2) {
                    if allow_proximity(&proxy1.data, &proxy2.data) {
                        let l1 = if proxy1.status == 0 {
                            &self.stree[proxy1.leaf]
                        }
                        else {
                            &self.tree[proxy1.leaf]
                        };

                        let l2 = if proxy2.status == 0 {
                            &self.stree[proxy2.leaf]
                        }
                        else {
                            &self.tree[proxy2.leaf]
                        };

                        if l1.bounding_volume.intersects(&l2.bounding_volume) {
                            remove = false
                        }
                    }
                }

                if remove {
                    let proxy1 = if let Some(proxy1) = proxy1 {
                        proxy1
                    } else {
                        &self.rem_proxies[ids.first.uid()]
                    };
                    let proxy2 = if let Some(proxy2) = proxy2 {
                        proxy2
                    } else {
                        &self.rem_proxies[ids.second.uid()]
                    };

                    handler(&proxy1.data, &proxy2.data, false);
                    self.pairs_to_remove.push(ids);
                }
            }

            self.update_off = if self.pairs.len() != 0 {
                (self.update_off + num_removals) % self.pairs.len()
            } else {
                0
            };
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
         * (for example, using a timestamp instead of a counter).
         */
        for (_, proxy) in self.proxies.iter_mut() {
            if proxy.status == 1 {
                proxy.status = 0;
                let leaf = self.tree.remove(proxy.leaf);
                proxy.leaf = self.stree.insert(leaf);
            }
            else if proxy.status > 1 {
                proxy.status -= 1;
            }
        }
    }
}

impl<P, BV, T> BroadPhase<P, BV, T> for DBVTBroadPhase<P, BV, T>
    where P:  Point,
          BV: BoundingVolume<P> + RayCast<P, Id> + PointQuery<P, Id> + Any + Send + Sync + Clone,
          T:  Any + Send + Sync {
    fn update(&mut self, allow_proximity: &mut FnMut(&T, &T) -> bool, handler: &mut FnMut(&T, &T, bool)) {
        /*
         * Remove from the trees all nodes that have been deleted or modified.
         */
        for (proxy_key, action) in self.actions.drain() {
            match action {
                Action::Modify(bv, data) => {
                    let proxy = &mut self.proxies[proxy_key];

                    if let Some(data) = data {
                        proxy.data = data;
                    }

                    match proxy.status() {
                        ProxyStatus::OnStaticTree => {
                            let mut leaf = self.stree.remove(proxy.leaf);
                            leaf.bounding_volume = bv;
                            self.to_update.push(leaf);
                        },
                        ProxyStatus::OnDynamicTree => {
                            let mut leaf = self.tree.remove(proxy.leaf);
                            leaf.bounding_volume = bv;
                            self.to_update.push(leaf);
                        },
                        ProxyStatus::Detached => {
                            let leaf = DBVTLeaf2::new(bv, proxy_key);
                            self.to_update.push(leaf);
                        }
                    }

                    proxy.detach();
                }
                Action::Remove(uid) => {
                    if let Some((_, proxy)) = self.proxies.remove(uid) {
                        match proxy.status() {
                            ProxyStatus::OnStaticTree => {
                                let _ = self.stree.remove(proxy.leaf);
                            }
                            ProxyStatus::OnDynamicTree => {
                                let _ = self.tree.remove(proxy.leaf);
                            }
                            ProxyStatus::Detached => { }
                        }
                        let replaced = self.rem_proxies.insert(uid, proxy);
                        assert!(replaced.1.is_none());
                    }
                }
            }
        }

        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        for leaf in self.to_update.drain(..) {
            {
                let proxy1 = &self.proxies[leaf.data];
                {
                    let mut visitor = BoundingVolumeInterferencesCollector::new(
                        &leaf.bounding_volume,
                        &mut self.collector);

                    self.tree.visit(&mut visitor);
                    self.stree.visit(&mut visitor);
                }

                // Event generation.
                for proxy_key2 in self.collector.iter() {
                    let proxy2 = &self.proxies[*proxy_key2];

                    if allow_proximity(&proxy1.data, &proxy2.data) {
                        let mut trigger = false;

                        let _ = self.pairs.find_or_insert_lazy(
                            Pair::new(leaf.data, *proxy_key2),
                            || { trigger = true; Some(()) });

                        if trigger {
                            handler(&proxy1.data, &proxy2.data, true)
                        }
                    }
                }

                self.collector.clear();
            }

            let proxy1 = &mut self.proxies[leaf.data];
            proxy1.leaf = self.tree.insert(leaf);
        }

        self.purge_some_contact_pairs(allow_proximity, handler);
        self.rem_proxies.clear();
        self.update_activation_states();
    }

    fn deferred_add(&mut self, uid: usize, bv: BV, data: T) {
        let proxy = DBVTBroadPhaseProxy::new(data);
        let (proxy_key, value) = self.proxies.insert_or_replace(uid, proxy, false);

        if let Some(proxy) = value {
            let _ = self.actions.insert(proxy_key, Action::Modify(bv, Some(proxy.data)));
        }
        else {
            let _ = self.actions.insert(proxy_key, Action::Modify(bv, None));
        }
    }

    fn deferred_remove(&mut self, uid: usize) {
        if let Some(proxy_key) = self.proxies.get_fast_key(uid) {
            let _ = self.actions.insert(proxy_key, Action::Remove(uid));
            self.purge_all = true;
        }
    }

    fn deferred_set_bounding_volume(&mut self, uid: usize, bounding_volume: BV) {
        if let Some(proxy_key) = self.proxies.get_fast_key(uid) {
            let proxy = &mut self.proxies[proxy_key];

            match self.actions.entry(proxy_key) {
                Entry::Occupied(mut entry) => {
                    match *entry.get_mut() {
                        Action::Modify(ref mut bv, _) => {
                            *bv = bounding_volume.loosened(self.margin);
                        },
                        Action::Remove(_) => { }
                    }
                },
                Entry::Vacant(entry) => {
                    let needs_update = match proxy.status() {
                        ProxyStatus::OnStaticTree  =>
                            !self.stree[proxy.leaf].bounding_volume.contains(&bounding_volume),
                        ProxyStatus::OnDynamicTree =>
                            !self.tree[proxy.leaf].bounding_volume.contains(&bounding_volume),
                        ProxyStatus::Detached      => true
                    };

                    if needs_update {
                        let new_bv = bounding_volume.loosened(self.margin);
                        let _ = entry.insert(Action::Modify(new_bv, None));
                    }
                }
            }
        }
    }

    fn deferred_recompute_all_proximities(&mut self) {
        for (proxy_key, proxy) in self.proxies.iter() {
            let bv;
            match proxy.status() {
                ProxyStatus::OnStaticTree  => { bv = &self.stree[proxy.leaf].bounding_volume; }
                ProxyStatus::OnDynamicTree => { bv = &self.tree[proxy.leaf].bounding_volume; }
                ProxyStatus::Detached      => continue
            }

            match self.actions.entry(proxy_key) {
                Entry::Vacant(entry) => {
                    // FIXME: too bad we have to clone the bounding volume…
                    let _  = entry.insert(Action::Modify(bv.clone(), None));
                },
                Entry::Occupied(_) => { }
            }
        }
    }

    fn interferences_with_bounding_volume<'a>(&'a self, bv: &BV, out: &mut Vec<&'a T>) {
        let mut collector = Vec::new();

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(bv, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l].data)
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
            out.push(&self.proxies[l].data)
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
            out.push(&self.proxies[l].data)
        }
    }
}
