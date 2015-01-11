use std::cell::RefCell;
use std::rc::Rc;
use na::Translation;
use na;
use utils::data::uid_remap::{UidRemap, FastKey};
use utils::data::pair::{Pair, PairTWHash};
use utils::data::hash_map::HashMap;
use math::{Scalar, Point, Vect};
use entities::bounding_volume::{HasBoundingVolume, BoundingVolume, BoundingVolumeInterferencesCollector};
use entities::partitioning::{DBVT, DBVTLeaf};
use queries::ray::{Ray, LocalRayCast, RayInterferencesCollector};
use queries::point::{LocalPointQuery, PointInterferencesCollector};
use broad_phase::BroadPhase;

struct DBVTBroadPhaseProxy<P, BV, T> {
    data:   T,
    leaf:   Rc<RefCell<DBVTLeaf<P, FastKey, BV>>>,
    active: isize // Negative => removed.
}

const DEACTIVATION_THRESHOLD: isize = 100;
const REMOVE_FROM_TREE: isize = -1;
const REMOVE_FROM_STREE: isize = -2;


/// Broad phase based on a Dynamic Bounding Volume Tree.
///
/// It uses two separate trees: one for static objects and which is never updated, and one for
/// moving objects.
pub struct DBVTBroadPhase<N, P, BV, T> {
    proxies:    UidRemap<DBVTBroadPhaseProxy<P, BV, T>>,
    tree:       DBVT<P, FastKey, BV>, // DBVT for moving objects.
    stree:      DBVT<P, FastKey, BV>, // DBVT for static objects.
    pairs:      HashMap<Pair, (), PairTWHash>, // Pairs detected (FIXME: use a Vec instead?)
    margin:     N, // The margin added to each bounding volume.
    update_off: usize, // Incremental pairs removal index.
    purge_all:  bool,

    // Just to avoid dynamic allocations.
    collector:  Vec<FastKey>,
    to_update:  Vec<(FastKey, BV)>,
}

#[old_impl_check]
impl<N, P, V, BV, T> DBVTBroadPhase<N, P, BV, T>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          BV: 'static + BoundingVolume<N> + Translation<V> + Clone {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(margin: N, small_keys: bool)
               -> DBVTBroadPhase<N, P, BV, T> {
        DBVTBroadPhase {
            proxies:    UidRemap::new(small_keys),
            tree:       DBVT::new(),
            stree:      DBVT::new(),
            pairs:      HashMap::new(PairTWHash::new()),
            update_off: 0,
            purge_all:  false,
            collector:  Vec::new(),
            to_update:  Vec::new(),
            margin:     margin
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> usize {
        self.pairs.len()
    }
}

impl<N, P, V, BV, T> BroadPhase<P, V, BV, T> for DBVTBroadPhase<N, P, BV, T>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          BV: 'static + BoundingVolume<N> + Translation<V> + LocalRayCast<N, P, V> + LocalPointQuery<N, P> + Clone {
    #[inline]
    fn defered_add(&mut self, uid: usize, bv: BV, data: T) {
        let lbv = bv.loosened(self.margin.clone());
        let leaf: DBVTLeaf<P, FastKey, BV> = DBVTLeaf::new(lbv.clone(), FastKey::new_invalid());
        let leaf = Rc::new(RefCell::new(leaf));
        let proxy = DBVTBroadPhaseProxy {
            data:   data,
            leaf:   leaf.clone(),
            active: DEACTIVATION_THRESHOLD
        };

        let (proxy_key, _) = self.proxies.insert(uid, proxy);
        leaf.borrow_mut().object = proxy_key.clone();
        self.to_update.push((proxy_key, lbv));
    }

    fn defered_remove(&mut self, uid: usize) {
        let proxy_key = match self.proxies.get_fast_key(uid) {
            None      => return,
            Some(uid) => uid
        };

        {
            let proxy = self.proxies.get_fast_mut(&proxy_key).unwrap();

            match proxy.active {
                // Already removed.
                REMOVE_FROM_TREE | REMOVE_FROM_STREE => { },
                // To remove from the static tree.
                0 => proxy.active = REMOVE_FROM_STREE,
                // To remove from the dynamic tree.
                _ => proxy.active = REMOVE_FROM_TREE
            }

            self.purge_all = true;
        }
    }

    fn update(&mut self, allow_proximity: &mut FnMut(&T, &T) -> bool, handler: &mut FnMut(&T, &T, bool)) {
        /*
         * Remove all the outdated nodes.
         */
        for &(ref id, ref bv) in self.to_update.iter().rev() {
            // FIXME: the `None` case may actually happen if the object is updated, then
            // removed, then the broad phase is updated.
            let proxy = self.proxies.get_fast_mut(id).expect("The proxy was not valid.");

            // If the activation number is < than the threshold then the leaf has not been removed
            // yet.
            if proxy.active == 0 {
                proxy.leaf.borrow_mut().bounding_volume = bv.clone();
                self.stree.remove(&mut proxy.leaf);
                proxy.active = DEACTIVATION_THRESHOLD;
            }
            else if proxy.active == REMOVE_FROM_TREE {
                self.tree.remove(&mut proxy.leaf);
            }
            else if proxy.active == REMOVE_FROM_STREE {
                self.stree.remove(&mut proxy.leaf);
            }
            else if proxy.active < DEACTIVATION_THRESHOLD {
                proxy.leaf.borrow_mut().bounding_volume = bv.clone();
                self.tree.remove(&mut proxy.leaf);
                proxy.active = DEACTIVATION_THRESHOLD;
            }
        }

        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        for &(ref proxy_key1, _) in self.to_update.iter().rev() {
            let proxy1 = &self.proxies[*proxy_key1];

            if !proxy1.leaf.borrow().is_detached() {
                continue;
            }

            if proxy1.active < 0 {
                continue;
            }

            {
                let node1 = proxy1.leaf.borrow();
                let mut visitor = BoundingVolumeInterferencesCollector::new(
                    &node1.bounding_volume,
                    &mut self.collector);

                self.tree.visit(&mut visitor);
                self.stree.visit(&mut visitor);
            }

            // Event generation.
            for proxy_key2 in self.collector.iter() {
                let proxy2 = &self.proxies[*proxy_key2];
                let filtered_out = proxy2.active < 0 || !allow_proximity(&proxy1.data, &proxy2.data);

                if !filtered_out {
                    let mut trigger = false;

                    let _ = self.pairs.find_or_insert_lazy(
                        Pair::new(*proxy_key1, *proxy_key2),
                        || { trigger = true; Some(()) });

                    if trigger {
                        handler(&proxy1.data, &proxy2.data, true)
                    }
                }
            }

            self.collector.clear();
            self.tree.insert(proxy1.leaf.clone());
        }

        /*
         * Update activation states.
         */
        for (_, proxy) in self.proxies.iter_mut() {
            if proxy.active == 1 {
                proxy.active = 0;
                self.tree.remove(&mut proxy.leaf);
                self.stree.insert(proxy.leaf.clone());
            }
            else if proxy.active > 1 {
                proxy.active = proxy.active - 1;
            }
        }

        /*
         * Remove some of the outdated collisions.
         */
        // NOTE: the exact same code is used on `brute_force_bounding_volume_broad_phase.rs`.
        // Refactor that?
        if self.purge_all || (self.to_update.len() != 0 && self.pairs.len() != 0) {
            let len          = self.pairs.len();
            let num_removals;
            
            if self.purge_all {
                self.update_off = 0;
                num_removals = len;
                self.purge_all = false;
            }
            else {
                num_removals = na::clamp(self.to_update.len(), len / 10, len);
            }

            for i in range(self.update_off, self.update_off + num_removals) {
                let id = i % self.pairs.len();

                let remove = {
                    let elts = self.pairs.elements();
                    let ids = elts[id].key;

                    let proxy1 = &self.proxies[ids.first];
                    let proxy2 = &self.proxies[ids.second];

                    let bv1 = &proxy1.leaf.borrow().bounding_volume;
                    let bv2 = &proxy2.leaf.borrow().bounding_volume;

                    let filtered_out = proxy1.active < 0 ||
                                       proxy2.active < 0 ||
                                       !allow_proximity(&proxy1.data, &proxy2.data);

                    if filtered_out || !bv1.intersects(bv2) {
                        handler(&proxy1.data, &proxy2.data, false);

                        true
                    }
                    else {
                        false
                    }
                };

                if remove {
                    self.pairs.remove_elem_at(id);
                }
            }

            self.update_off = (self.update_off + num_removals) % self.pairs.len();
        }

        self.to_update.clear();
    }

    fn defered_set_bounding_volume(&mut self, uid: usize, bounding_volume: BV) {
        if let Some(proxy_key) = self.proxies.get_fast_key(uid) {
            let proxy = self.proxies.get_fast_mut(&proxy_key).unwrap();

            if proxy.active >= 0 {
                let needs_update = !proxy.leaf.borrow().bounding_volume.contains(&bounding_volume);

                if needs_update {
                    self.to_update.push((proxy_key, bounding_volume.loosened(self.margin)));
                }
                else if proxy.active != DEACTIVATION_THRESHOLD { // If == the object might already be on the update list.
                    if proxy.active == 0 {
                        self.stree.remove(&mut proxy.leaf);
                        self.tree.insert(proxy.leaf.clone());
                    }

                    proxy.active = DEACTIVATION_THRESHOLD - 1;
                }
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

    fn interferences_with_ray<'a>(&'a self, ray: &Ray<P, V>, out: &mut Vec<&'a T>) {
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
