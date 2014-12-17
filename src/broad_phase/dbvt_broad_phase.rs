use std::cell::RefCell;
use std::rc::Rc;
use na::Translation;
use na;
use utils::data::has_uid::HasUid;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::hash_map::HashMap;
use bounding_volume::{HasBoundingVolume, BoundingVolume};
use ray::{Ray, LocalRayCast};
use point::LocalPointQuery;
use partitioning::{DBVT, DBVTLeaf,
                   BoundingVolumeInterferencesCollector,
                   RayInterferencesCollector,
                   PointInterferencesCollector};
use math::{Scalar, Point, Vect};
use broad_phase::{BroadPhase, ProximityFilter, ProximitySignal, ProximitySignalHandler};
use utils::data::has_uid_map::{HasUidMap, FastKey};

struct DBVTBroadPhaseProxy<P, BV> {
    leaf:   Rc<RefCell<DBVTLeaf<P, FastKey, BV>>>,
    active: uint
}

const DEACTIVATION_THRESHOLD: uint = 100;


/// Broad phase based on a Dynamic Bounding Volume Tree.
///
/// It uses two separate trees: one for static objects and which is never updated, and one for
/// moving objects.
pub struct DBVTBroadPhase<N, P, B, BV> {
    proxies:    HasUidMap<B, DBVTBroadPhaseProxy<P, BV>>,
    tree:       DBVT<P, FastKey, BV>, // DBVT for moving objects.
    stree:      DBVT<P, FastKey, BV>, // DBVT for static objects.
    filter:     Option<Box<ProximityFilter<B> + 'static>>,
    pairs:      HashMap<Pair<FastKey>, (), PairTWHash>, // Pairs detected (FIXME: use a Vec instead?)
    signal:     ProximitySignal<B>, // To notify the user when a new pair is found/destroyed.
    margin:     N, // The margin added to each bounding volume.
    update_off: uint, // Incremental pairs removal index.

    // Just to avoid dynamic allocations.
    collector:  Vec<FastKey>,
    to_update:  Vec<(FastKey, BV)>,
}

impl<N, P, V, B, BV> DBVTBroadPhase<N, P, B, BV>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          B:  'static + HasUid,
          BV: 'static + BoundingVolume<N> + Translation<V> + Clone {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(margin: N, filter: Option<Box<ProximityFilter<B> + 'static>>)
               -> DBVTBroadPhase<N, P, B, BV> {
        DBVTBroadPhase {
            proxies:    HasUidMap::new(),
            tree:       DBVT::new(),
            stree:      DBVT::new(),
            filter:     filter,
            pairs:      HashMap::new(PairTWHash::new()),
            signal:     ProximitySignal::new(),
            update_off: 0,
            collector:  Vec::new(),
            to_update:  Vec::new(),
            margin:     margin
        }
    }

    /// Number of interferences detected by this broad phase.
    #[inline]
    pub fn num_interferences(&self) -> uint {
        self.pairs.len()
    }
}

impl<N, P, V, B, BV> BroadPhase<P, V, B, BV> for DBVTBroadPhase<N, P, B, BV>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          B:  'static + HasUid,
          BV: 'static + BoundingVolume<N> + Translation<V> + LocalRayCast<N, P, V> + LocalPointQuery<N, P> + Clone {
    #[inline]
    fn add(&mut self, b: B, bv: BV) {
        let lbv = bv.loosened(self.margin.clone());
        let leaf: DBVTLeaf<P, FastKey, BV> = DBVTLeaf::new(lbv.clone(), FastKey::new_invalid());
        let leaf = Rc::new(RefCell::new(leaf));
        let proxy = DBVTBroadPhaseProxy {
            leaf:   leaf.clone(),
            active: DEACTIVATION_THRESHOLD
        };

        let (proxy_key, _) = self.proxies.insert(b, proxy);
        leaf.borrow_mut().object = proxy_key.clone();
        self.to_update.push((proxy_key, lbv));
        self.update();
    }

    fn remove_with_uid(&mut self, uid: uint) {
        let proxy_key = match self.proxies.get_fast_key_with_uid(uid) {
            None      => return,
            Some(uid) => uid
        };

        {
            let proxy = self.proxies.get_fast_mut(&proxy_key).unwrap().1;
            if proxy.active != 0 {
                self.tree.remove(&mut proxy.leaf)
            }
            else {
                self.stree.remove(&mut proxy.leaf)
            }
        }

        let mut keys_to_remove = Vec::new();

        // Remove every pair involving b.
        for elt in self.pairs.elements().iter() {
            if elt.key.first == proxy_key || elt.key.second == proxy_key {
                keys_to_remove.push(elt.key.clone());
            }
        }

        // Remove and trigger the proximity loss signal.
        for k in keys_to_remove.iter() {
            self.pairs.remove(k);

            let o1 = &self.proxies[k.first].0;
            let o2 = &self.proxies[k.second].0;
            self.signal.trigger_proximity_signal(o1, o2, false);
        }

        keys_to_remove.clear();
        let _ = self.proxies.remove_with_uid(uid);
    }

    fn update(&mut self) {
        /*
         * Remove all the outdated nodes.
         */
        for &(ref id, ref bv)in self.to_update.iter().rev() {
            // FIXME: the `None` case may actually happen if the object is updated, then
            // removed, then the broad phase is updated.
            let proxy = self.proxies.get_fast_mut(id).expect("The proxy was not valid.").1;

            // If the activation number is < than the threshold then the leaf has not been removed
            // yet.
            if proxy.active == 0 {
                proxy.leaf.borrow_mut().bounding_volume = bv.clone();
                self.stree.remove(&mut proxy.leaf);
            }
            else if proxy.active < DEACTIVATION_THRESHOLD {
                proxy.leaf.borrow_mut().bounding_volume = bv.clone();
                self.tree.remove(&mut proxy.leaf);
            }

            proxy.active = DEACTIVATION_THRESHOLD;
        }

        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        for &(ref proxy_key1, _) in self.to_update.iter() {
            let &(ref object1, ref proxy1) = &self.proxies[*proxy_key1];

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
            let object2 = &self.proxies[*proxy_key2].0;
                // Do not trigger if the proximity event does not exist yet.

                let filtered_out = match self.filter {
                    None        => false,
                    Some(ref f) => !f.is_proximity_allowed(object1, object2)
                };

                if !filtered_out {
                    let mut trigger = false;

                    let _ = self.pairs.find_or_insert_lazy(
                        Pair::new(*proxy_key1, *proxy_key2),
                        || { trigger = true; Some(()) });

                    if trigger {
                        self.signal.trigger_proximity_signal(object1, object2, true)
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
        if self.to_update.len() != 0 && self.pairs.len() != 0 {
            let len          = self.pairs.len();
            let num_removals = na::clamp(self.to_update.len(), len / 10, len);

            for i in range(self.update_off, self.update_off + num_removals) {
                let id = i % self.pairs.len();

                let remove = {
                    let elts = self.pairs.elements();
                    let ids = &elts[id].key;

                    let &(ref object1, ref proxy1) = &self.proxies[ids.first];
                    let &(ref object2, ref proxy2) = &self.proxies[ids.second];

                    let bv1 = &proxy1.leaf.borrow().bounding_volume;
                    let bv2 = &proxy2.leaf.borrow().bounding_volume;

                    let filtered_out = match self.filter {
                        None        => false,
                        Some(ref f) => !f.is_proximity_allowed(object1, object2)
                    };

                    if filtered_out || !bv1.intersects(bv2) {
                        self.signal.trigger_proximity_signal(object1, object2, false);

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

    fn set_next_bounding_volume_with_uid(&mut self, uid: uint, bounding_volume: BV) {
        if let Some(proxy_key) = self.proxies.get_fast_key_with_uid(uid) {
            let proxy = self.proxies.get_fast_mut(&proxy_key).unwrap().1;
            let needs_update = !proxy.leaf.borrow().bounding_volume.contains(&bounding_volume);

            if needs_update {
                self.to_update.push((proxy_key, bounding_volume.loosened(self.margin)));
            }
            else {
                if proxy.active == 0 {
                    self.stree.remove(&mut proxy.leaf);
                    self.tree.insert(proxy.leaf.clone());
                }

                proxy.active = DEACTIVATION_THRESHOLD - 1;
            }
        }
    }

    fn register_proximity_signal_handler(&mut self, name: &str, handler: Box<ProximitySignalHandler<B> + 'static>) {
        self.signal.register_proximity_signal_handler(name, handler)
    }

    fn unregister_proximity_signal_handler(&mut self, name: &str) {
        self.signal.unregister_proximity_signal_handler(name)
    }

    fn interferences_with_bounding_volume<'a>(&'a self, bv: &BV, out: &mut Vec<&'a B>) {
        let mut collector = Vec::new();

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(bv, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l].0)
        }
    }

    fn interferences_with_ray<'a>(&'a self, ray: &Ray<P, V>, out: &mut Vec<&'a B>) {
        let mut collector = Vec::new();

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l].0)
        }
    }

    fn interferences_with_point<'a>(&'a self, point: &P, out: &mut Vec<&'a B>) {
        let mut collector = Vec::new();

        {
            let mut visitor = PointInterferencesCollector::new(point, &mut collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in collector.into_iter() {
            out.push(&self.proxies[l].0)
        }
    }
}
