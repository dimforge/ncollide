use std::gc::Gc;
use std::cell::RefCell;
use nalgebra::na::Translation;
use broad::{BroadPhase, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use partitioning::dbvt::{DBVT, DBVTLeaf};
use util::hash::UintTWHash;
use util::hash_map::HashMap;
use util::pair::{Pair, PairTWHash};
use util::has_uid::HasUid;
use broad::Dispatcher;
use bounding_volume::{HasBoundingVolume, LooseBoundingVolume};
use ray::{Ray, RayCast};
use partitioning::bvt_visitor::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};
use math::{N, V};

/// Broad phase based on a Dynamic Bounding Volume Tree. It uses two separate trees: one for static
/// objects and which is never updated, and one for moving objects.
pub struct DBVTBroadPhase<B, BV, D, DV> {
    priv tree:        DBVT<B, BV>,
    priv stree:       DBVT<B, BV>,
    priv active2bv:   HashMap<uint, Gc<RefCell<DBVTLeaf<B, BV>>>, UintTWHash>,
    priv inactive2bv: HashMap<uint, Gc<RefCell<DBVTLeaf<B, BV>>>, UintTWHash>,
    priv pairs:       HashMap<Pair<Gc<RefCell<DBVTLeaf<B, BV>>>>, DV, PairTWHash>, // pair manager
    priv spairs:      HashMap<Pair<Gc<RefCell<DBVTLeaf<B, BV>>>>, DV, PairTWHash>,
    priv dispatcher:  D,
    priv margin:      N,
    priv collector:   ~[Gc<RefCell<DBVTLeaf<B, BV>>>],
    priv to_update:   ~[Gc<RefCell<DBVTLeaf<B, BV>>>],
    priv update_off:  uint // incremental pairs removal index
}

impl<B:  'static + HasBoundingVolume<BV> + Clone,
     BV: 'static + LooseBoundingVolume + Translation<V> + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
DBVTBroadPhase<B, BV, D, DV> {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(dispatcher: D, margin: N) -> DBVTBroadPhase<B, BV, D, DV> {
        DBVTBroadPhase {
            tree:        DBVT::new(),
            stree:       DBVT::new(),
            active2bv:   HashMap::new(UintTWHash::new()),
            inactive2bv: HashMap::new(UintTWHash::new()),
            pairs:       HashMap::new(PairTWHash::new()),
            spairs:      HashMap::new(PairTWHash::new()),
            dispatcher:  dispatcher,
            update_off:  0,
            collector:   ~[],
            to_update:   ~[],
            margin:      margin
        }
    }

    /// Number of interferences detected by this broad phase.
    pub fn num_interferences(&self) -> uint {
        self.pairs.len()
    }

    fn update_updatable(&mut self) {
        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        let mut new_colls = 0u;

        for u in self.to_update.iter() {
            { // scope to avoid dynamic borrow failur.
                let bu = u.borrow().borrow();
                self.tree.interferences_with_leaf(bu.get(), &mut self.collector);
                self.stree.interferences_with_leaf(bu.get(), &mut self.collector);

                // dispatch
                for i in self.collector.iter() {
                    let bi = i.borrow().borrow();
                    if self.dispatcher.is_valid(&bu.get().object, &bi.get().object) {
                        self.pairs.find_or_insert_lazy(
                            Pair::new(u.clone(), i.clone()),
                            || self.dispatcher.dispatch(&bu.get().object, &bi.get().object)
                            );

                        new_colls = new_colls + 1;
                    }
                }
            }

            self.collector.clear();
            self.tree.insert(*u);
        }

        /*
         * Remove outdated collisions
         */
        // NOTE: the exact same code is used on `brute_force_bounding_volume_broad_phase.rs`.
        // Refactor that?
        if new_colls != 0 {
            let len          = self.pairs.len();
            let num_removals = new_colls.clamp(&(len / 10), &len);

            for i in range(self.update_off, self.update_off + num_removals) {
                let id = i % self.pairs.len();

                let remove = {
                    let elts  = self.pairs.elements();
                    let entry = &elts[id];

                    let bf = entry.key.first.borrow().borrow();
                    let bs = entry.key.second.borrow().borrow();
                    if (!bf.get().bounding_volume.intersects(&bs.get().bounding_volume)) {
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
}

impl<B:  'static + HasBoundingVolume<BV> + Clone + HasUid,
     BV: 'static + LooseBoundingVolume + Translation<V> + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
BroadPhase<B> for DBVTBroadPhase<B, BV, D, DV> {
    #[inline]
    fn add(&mut self, b: B) {
        let id   = b.uid();
        let leaf = Gc::new(RefCell::new(DBVTLeaf::new(b.bounding_volume().loosened(self.margin.clone()), b)));

        self.to_update.push(leaf);
        self.update_updatable();

        self.active2bv.insert(id, leaf);
    }

    fn remove(&mut self, b: &B) {
        // remove b from the dbvts
        let key      = b.uid();
        let leaf_opt = self.active2bv.get_and_remove(&key);
        let mut leaf;

        match leaf_opt {
            Some(l) => {
                leaf = l.value;
                self.tree.remove(&mut leaf);
            },
            None => {
                let leaf_opt = self.inactive2bv.get_and_remove(&key);
                match leaf_opt {
                    Some(l) => {
                        leaf = l.value;
                        self.stree.remove(&mut leaf);
                    },
                    None => return
                }
            }
        }

        let mut keys_to_remove = ~[];

        // remove every pair involving b
        for elt in self.pairs.elements().iter() {
            if elt.key.first.uid() == leaf.uid() || elt.key.second.uid() == leaf.uid() {
                keys_to_remove.push(elt.key);
            }
        }

        for k in keys_to_remove.iter() {
            self.pairs.remove(k);
        }

        keys_to_remove.clear();

        // remove every "sleeping" pair involving b
        for elt in self.spairs.elements().iter() {
            if elt.key.first.uid() == leaf.uid() || elt.key.second.uid() == leaf.uid() {
                keys_to_remove.push(elt.key);
            }
        }

        for k in keys_to_remove.iter() {
            self.spairs.remove(k);
        }
    }

    fn update(&mut self) {
        // NOTE: be careful not to add the same object twice!
        /*
         * Remove all outdated nodes
         */
        for a in self.active2bv.elements_mut().mut_iter() {
            let mut new_bv = a.value.borrow().with(|o| o.object.bounding_volume());

            if !a.value.borrow().with(|v| v.bounding_volume.contains(&new_bv)) {
                // need an update!
                new_bv.loosen(self.margin.clone());

                {
                    let mut bv = a.value.borrow().borrow_mut();
                    bv.get().bounding_volume = new_bv;
                }

                self.tree.remove(&mut a.value);
                self.to_update.push(a.value);
            }
        }

        self.update_updatable();
    }

    fn update_object(&mut self, object: &B) {
        match self.active2bv.find_mut(&object.uid()) {
            None       => { },
            Some(leaf) => {
                let mut new_bv = leaf.borrow().with(|l| l.object.bounding_volume());
                if !leaf.borrow().with(|l| l.bounding_volume.contains(&new_bv)) {
                    // update for real
                    new_bv.loosen(self.margin.clone());
                    {
                        let mut bl = leaf.borrow().borrow_mut();
                        bl.get().bounding_volume = new_bv;
                    }
                    self.tree.remove(leaf);
                    self.to_update.push(leaf.clone());
                }
            }
        }

        self.update_updatable();
    }
}

impl<B:  'static + HasBoundingVolume<BV> + HasUid + Clone,
     BV: 'static + LooseBoundingVolume + Translation<V> + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
InterferencesBroadPhase<B, DV> for DBVTBroadPhase<B, BV, D, DV> {
    #[inline(always)]
    fn for_each_pair(&self, f: |&B, &B, &DV| -> ()) {
        for p in self.pairs.elements().iter() {
            let mut bf = p.key.first.borrow().borrow_mut();
            let mut bs = p.key.second.borrow().borrow_mut();
            f(&bf.get().object, &bs.get().object, &p.value)
        }
    }

    #[inline(always)]
    fn for_each_pair_mut(&mut self, f: |&B, &B, &mut DV| -> ()) {
        for p in self.pairs.elements_mut().mut_iter() {
            let bf = p.key.first.borrow().borrow();
            let bs = p.key.second.borrow().borrow();
            f(&bf.get().object, &bs.get().object, &mut p.value)
        }
    }

    #[inline(always)]
    fn activate(&mut self, body: &B, f: |&B, &B, &mut DV| -> ()) {
        // verify that it is not already active and add it to the active map.
        let mut leaf =
            match self.inactive2bv.get_and_remove(&body.uid()) {
                None    => return, // not found: the object is already active
                Some(l) => l.value
            };

        self.active2bv.insert(body.uid(), leaf);

        // remove from the inactive tree
        self.stree.remove(&mut leaf);

        // Now we find interferences with inactive objects.
        { // scope to avoid dynamic borrow failure
            let bleaf = leaf.borrow().borrow();
            self.stree.interferences_with_leaf(bleaf.get(), &mut self.collector);

            for i in self.collector.iter() {
                let bi = i.borrow().borrow();
                if self.dispatcher.is_valid(&bleaf.get().object, &bi.get().object) {
                    // the intereference should be registered on the spairs already
                    match self.spairs.get_and_remove(&Pair::new(leaf, *i)) {
                        Some(dv) => {
                            let bdvf = dv.key.first.borrow().borrow();
                            let bdvs = dv.key.second.borrow().borrow();
                            let obj1 = &bdvf.get().object;
                            let obj2 = &bdvs.get().object;
                            let p    = self.pairs.insert_or_replace(dv.key, dv.value, true);

                            f(obj1, obj2, p)
                        },
                        None => fail!("Internal error: found a new collision during the activation.")
                    }
                }
            }
        }

        // add to the active tree
        self.tree.insert(leaf);
        self.collector.clear();
    }

    fn deactivate(&mut self, body: &B) {
        // verify that it is not already inactive and add it to the inactive map.
        let mut leaf =
            match self.active2bv.get_and_remove(&body.uid()) {
                None    => return, // not found: the object is already inactive
                Some(l) => l.value
            };

        self.inactive2bv.insert(body.uid(), leaf);

        // Now transfer all collisions involving `leaf` and deactivated objects from `pairs` to
        // `spairs`.

        // remove from the active tree
        self.tree.remove(&mut leaf);

        { // scope to avoid dynamic borrow failure of leaf
            let bleaf = leaf.borrow().borrow();
            self.stree.interferences_with_leaf(bleaf.get(), &mut self.collector);

            for i in self.collector.iter() {
                let fi = i.borrow().borrow();
                if self.dispatcher.is_valid(&bleaf.get().object, &fi.get().object) {
                    // the intereference should be registered on the pairs already
                    match self.pairs.get_and_remove(&Pair::new(leaf, *i)) {
                        Some(dv) => { self.spairs.insert(dv.key, dv.value); },
                        None     => fail!("Internal error: found a new collision during the deactivation.")
                    }
                }
            }
        }

        // add to the inactive tree
        self.stree.insert(leaf);
        self.collector.clear();
    }
}

impl<B:  'static + HasBoundingVolume<BV> + HasUid + Clone,
     BV: 'static + LooseBoundingVolume + Translation<V> + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
BoundingVolumeBroadPhase<B, BV> for DBVTBroadPhase<B, BV, D, DV> {
    fn interferences_with_bounding_volume(&mut self, bv: &BV, out: &mut ~[B]) {
        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(bv, &mut self.collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in self.collector.iter() {
            l.borrow().with(|l| out.push(l.object.clone()))
        }

        self.collector.clear()
    }
}

impl<B:  'static + HasBoundingVolume<BV> + HasUid + Clone,
     BV: 'static + LooseBoundingVolume + RayCast + Translation<V> + Clone,
     D:  Dispatcher<B, B, DV>,
     DV>
RayCastBroadPhase<B> for DBVTBroadPhase<B, BV, D, DV> {
    fn interferences_with_ray(&mut self, ray: &Ray, out: &mut ~[B]) {
        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut self.collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in self.collector.iter() {
            l.borrow().with(|l| out.push(l.object.clone()))
        }

        self.collector.clear()
    }
}

#[cfg(test, dim3, f64)]
mod test {
    use super::DBVTBroadPhase;
    use std::rc::Rc;
    use std::cell::RefCell;
    use nalgebra::na::{Vec3, Iso3};
    use nalgebra::na;
    use geom::Ball;
    use bounding_volume::WithAABB;
    use broad::{NoIdDispatcher, BroadPhase, InterferencesBroadPhase};

    // #[test]
    // fn test_dbvt_empty() {
    //     type Shape = Rc<WithAABB<Ball>>;
    //     let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
    //     let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
    //     let ball       = Ball::new(0.3);

    //     for i in range(-10, 10) {
    //         for j in range(-10, 10) {
    //             let t = Vec3::new(i as f64 * 30.0, j as f64 * 30.0, 0.0);
    //             bf.add(Rc::new(WithAABB(Iso3::new(t, na::zero()), ball)));
    //         }
    //     }

    //     bf.update();

    //     assert_eq!(bf.num_interferences(), 0)
    // }

    // #[test]
    // fn test_dbvt_nbh_collide() {
    //     type Shape = Rc<WithAABB<Ball>>;
    //     let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
    //     let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
    //     let ball       = Ball::new(0.3);

    //     // create a grid
    //     for i in range(-10, 10) {
    //         for j in range(-10, 10) {
    //             let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
    //             bf.add(Rc::new(WithAABB(Iso3::new(t, na::zero()), ball)));
    //         }
    //     }

    //     bf.update();

    //     assert_eq!(
    //         bf.num_interferences(),
    //         (18 * 18 * 8 + // internal rectangles have 8 neighbors
    //          18 * 4 * 5  + // border (excluding corners) rectangles have 5 neighbors
    //          4 * 3)        // corners have 3 neighbors
    //         / 2            // remove all duplicates
    //     )
    // }

    #[test]
    fn test_dbvt_nbh_move_collide() {
        type Shape = Rc<RefCell<WithAABB<Ball>>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        let mut to_move = ~[];

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let t = Vec3::new(i as f64 * 0.9, j as f64 * 0.9, 0.0);
                let to_add = Rc::from_mut(RefCell::new(WithAABB(Iso3::new(t, na::zero()), ball)));
                bf.add(to_add.clone());
                to_move.push(to_add);
            }
        }

        bf.update();

        // test deactivations followed by activations: this should not changes anything
        for e in to_move.mut_iter() {
            bf.deactivate(e);
        }

        // nothing on pairs …
        assert_eq!(bf.num_interferences(), 0);
        // … because it has been transfered to spairs. NOTE: `spairs` is used for test only here,
        // there is no explicit way for the user to access it.

        for e in to_move.mut_iter() {
            bf.activate(e, |_, _, _| { });
        }

        // test one deactivation followed by one activation: this should not change anything
        for e in to_move.mut_iter() {
            bf.deactivate(e);
            bf.activate(e, |_, _, _| { });
        }

        for e in to_move.mut_iter() {
            let mut wa = e.borrow().borrow_mut();
            let m = wa.get().m().clone();
            let g = wa.get().g().clone();
            *wa.get() = WithAABB(na::append_translation(&m, &Vec3::new(10.0, 10.0, 10.0)), g)
        }

        bf.update();

        assert_eq!(
            bf.num_interferences(),
            (18 * 18 * 8 + // internal rectangles have 8 neighbors
             18 * 4 * 5  + // border (excluding corners) rectangles have 5 neighbors
             4 * 3)        // corners have 3 neighbors
             / 2           // remove all duplicates
        )
    }

    // #[test]
    // fn test_dbvt_quadratic_collide() {
    //     type Shape = Rc<WithAABB<Ball>>;
    //     let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
    //     let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
    //     let ball       = Ball::new(0.3);

    //     400.times(|| {
    //         bf.add(Rc::new(WithAABB(Iso3::new(na::zero(), na::zero()), ball)))
    //     });

    //     bf.update();

    //     assert_eq!(bf.num_interferences(), (399 * (399 + 1)) / 2)
    // }
}
