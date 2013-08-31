use std::ptr;
use nalgebra::mat::Translation;
use nalgebra::vec::AlgebraicVec;
use broad::{BroadPhase, InterferencesBroadPhase, BoundingVolumeBroadPhase, RayCastBroadPhase};
use partitioning::dbvt::{DBVT, DBVTLeaf};
use util::hash::UintTWHash;
use util::hash_map::HashMap;
use util::pair::{Pair, PairTWHash};
use broad::Dispatcher;
use bounding_volume::{HasBoundingVolume, LooseBoundingVolume};
use ray::{Ray, RayCast};
use partitioning::bvt_visitor::{BoundingVolumeInterferencesCollector, RayInterferencesCollector};

/// Broad phase based on a Dynamic Bounding Volume Tree. It uses two separate trees: one for static
/// objects and which is never updated, and one for moving objects.
pub struct DBVTBroadPhase<N, V, B, BV, D, DV> {
    priv tree:        DBVT<V, @mut B, BV>,
    priv stree:       DBVT<V, @mut B, BV>,
    priv active2bv:   HashMap<uint, @mut DBVTLeaf<V, @mut B, BV>, UintTWHash>,
    priv inactive2bv: HashMap<uint, @mut DBVTLeaf<V, @mut B, BV>, UintTWHash>,
    priv pairs:       HashMap<Pair<DBVTLeaf<V, @mut B, BV>>, DV, PairTWHash>, // pair manager
    priv spairs:      HashMap<Pair<DBVTLeaf<V, @mut B, BV>>, DV, PairTWHash>,
    priv dispatcher:  D,
    priv margin:      N,
    priv collector:   ~[@mut DBVTLeaf<V, @mut B, BV>],
    priv to_update:   ~[@mut DBVTLeaf<V, @mut B, BV>],
    priv update_off:  uint // incremental pairs removal index
}

impl<N:  Algebraic + Clone + Ord,
     V:  'static + AlgebraicVec<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
DBVTBroadPhase<N, V, B, BV, D, DV> {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(dispatcher: D, margin: N) -> DBVTBroadPhase<N, V, B, BV, D, DV> {
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
            self.tree.interferences_with_leaf(*u, &mut self.collector);
            self.stree.interferences_with_leaf(*u, &mut self.collector);

            // dispatch
            for i in self.collector.iter() {
                if self.dispatcher.is_valid(u.object, i.object) {
                    self.pairs.find_or_insert_lazy(
                        Pair::new(*u, *i),
                        || self.dispatcher.dispatch(u.object, i.object)
                        );

                    new_colls = new_colls + 1;
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

                    if (!entry.key.first.bounding_volume.intersects(&entry.key.second.bounding_volume)) {
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

impl<N:  Algebraic + Clone + Ord,
     V:  'static + AlgebraicVec<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
BroadPhase<B> for DBVTBroadPhase<N, V, B, BV, D, DV> {
    #[inline]
    fn add(&mut self, rb: @mut B) {
        let leaf = @mut DBVTLeaf::new(rb.bounding_volume().loosened(self.margin.clone()), rb);

        self.to_update.push(leaf);
        self.update_updatable();

        self.active2bv.insert(ptr::to_mut_unsafe_ptr(rb) as uint, leaf);
    }

    fn remove(&mut self, _: @mut B) {
        fail!("Not yet implemented.");
    }

    fn update(&mut self) {
        // NOTE: be careful not to add the same object twice!
        /*
         * Remove all outdated nodes
         */
        for a in self.active2bv.elements().iter() {
            let mut new_bv = a.value.object.bounding_volume();

            if !a.value.bounding_volume.contains(&new_bv) {
                // need an update!
                new_bv.loosen(self.margin.clone());
                a.value.bounding_volume = new_bv;
                self.tree.remove(a.value);
                self.to_update.push(a.value);
            }
        }

        self.update_updatable();
    }

    fn update_object(&mut self, object: @mut B) {
        match self.active2bv.find(&(ptr::to_mut_unsafe_ptr(object) as uint)) {
            None       => { },
            Some(leaf) => {
                let mut new_bv = leaf.object.bounding_volume();
                if !leaf.bounding_volume.contains(&new_bv) {
                    // update for real
                    new_bv.loosen(self.margin.clone());
                    leaf.bounding_volume = new_bv;
                    self.tree.remove(*leaf);
                    self.to_update.push(*leaf);
                }
            }
        }

        self.update_updatable();
    }
}

impl<N:  Algebraic + Clone + Ord,
     V:  'static + AlgebraicVec<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
InterferencesBroadPhase<B, DV> for DBVTBroadPhase<N, V, B, BV, D, DV> {
    #[inline(always)]
    fn for_each_pair(&self, f: &fn(@mut B, @mut B, &DV)) {
        for p in self.pairs.elements().iter() {
            f(p.key.first.object, p.key.second.object, &p.value)
        }
    }

    #[inline(always)]
    fn for_each_pair_mut(&mut self, f: &fn(@mut B, @mut B, &mut DV)) {
        for p in self.pairs.elements_mut().mut_iter() {
            f(p.key.first.object, p.key.second.object, &mut p.value)
        }
    }

    #[inline(always)]
    fn activate(&mut self, body: @mut B, f: &fn(@mut B, @mut B, &mut DV)) {
        // verify that it is not already active and add it to the active map.
        let leaf =
            match self.inactive2bv.get_and_remove(&(ptr::to_mut_unsafe_ptr(body) as uint)) {
                None    => return, // not found: the object is already active
                Some(l) => l.value
            };

        self.active2bv.insert(ptr::to_mut_unsafe_ptr(body) as uint, leaf);

        // remove from the inactive tree
        self.stree.remove(leaf);

        // Now we find interferences with inactive objects.
        self.stree.interferences_with_leaf(leaf, &mut self.collector);

        for i in self.collector.iter() {
            if self.dispatcher.is_valid(leaf.object, i.object) {
                // the intereference should be registered on the spairs already
                match self.spairs.get_and_remove(&Pair::new(leaf, *i)) {
                    Some(dv) => {
                        let obj1 = dv.key.first.object;
                        let obj2 = dv.key.second.object;
                        let p    = self.pairs.insert_or_replace(dv.key, dv.value, true);

                        f(obj1, obj2, p)
                    },
                    None => fail!("Internal error: found a new collision during the activation.")
                }
            }
        }

        // add to the active tree
        self.tree.insert(leaf);
        self.collector.clear();
    }

    fn deactivate(&mut self, body: @mut B) {
        // verify that it is not already inactive and add it to the inactive map.
        let leaf =
            match self.active2bv.get_and_remove(&(ptr::to_mut_unsafe_ptr(body) as uint)) {
                None    => return, // not found: the object is already inactive
                Some(l) => l.value
            };

        self.inactive2bv.insert(ptr::to_mut_unsafe_ptr(body) as uint, leaf);

        // Now transfer all collisions involving `leaf` and deactivated objects from `pairs` to
        // `spairs`.

        // remove from the active tree
        self.tree.remove(leaf);

        self.stree.interferences_with_leaf(leaf, &mut self.collector);

        for i in self.collector.iter() {
            if self.dispatcher.is_valid(leaf.object, i.object) {
                // the intereference should be registered on the pairs already
                match self.pairs.get_and_remove(&Pair::new(leaf, *i)) {
                    Some(dv) => { self.spairs.insert(dv.key, dv.value); },
                    None     => fail!("Internal error: found a new collision during the deactivation.")
                }
            }
        }

        // add to the inactive tree
        self.stree.insert(leaf);
        self.collector.clear();
    }
}

impl<N:  Algebraic + Clone + Ord,
     V:  'static + AlgebraicVec<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
BoundingVolumeBroadPhase<B, BV> for DBVTBroadPhase<N, V, B, BV, D, DV> {
    fn interferences_with_bounding_volume(&mut self, bv: &BV, out: &mut ~[@mut B]) {
        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(bv, &mut self.collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in self.collector.iter() {
            out.push(l.object)
        }

        self.collector.clear()
    }
}

impl<N:  Algebraic + Clone + Ord,
     V:  'static + AlgebraicVec<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + RayCast<N, V> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
RayCastBroadPhase<V, B> for DBVTBroadPhase<N, V, B, BV, D, DV> {
    fn interferences_with_ray(&mut self, ray: &Ray<V>, out: &mut ~[@mut B]) {
        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut self.collector);

            self.tree.visit(&mut visitor);
            self.stree.visit(&mut visitor);
        }

        for l in self.collector.iter() {
            out.push(l.object)
        }

        self.collector.clear()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::vec::Vec3;
    use geom::Ball;
    use bounding_volume::WithAABB;
    use broad::NoIdDispatcher;

    #[test]
    fn test_dbvt_empty() {
        type Shape = WithAABB<Vec3<float>, Ball<float>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        for i in range(-10, 10) {
            for j in range(-10, 10) {
                bf.add(@mut WithAABB(Vec3::new(i as float * 30.0, j as float * 30.0, 0.0), ball));
            }
        }

        bf.update();

        assert_eq!(bf.num_interferences(), 0)
    }

    #[test]
    fn test_dbvt_nbh_collide() {
        type Shape = WithAABB<Vec3<float>, Ball<float>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                bf.add(@mut WithAABB(Vec3::new(i as float * 0.9, j as float * 0.9, 0.0), ball));
            }
        }

        bf.update();

        assert_eq!(
            bf.num_interferences(),
            (18 * 18 * 8 + // internal rectangles have 8 neighbors
             18 * 4 * 5  + // border (excluding corners) rectangles have 5 neighbors
             4 * 3)        // corners have 3 neighbors
            / 2            // remove all duplicates
        )
    }

    #[test]
    fn test_dbvt_nbh_move_collide() {
        type Shape = WithAABB<Vec3<float>, Ball<float>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        let mut to_move = ~[];

        // create a grid
        for i in range(-10, 10) {
            for j in range(-10, 10) {
                let to_add = @mut WithAABB(Vec3::new(i as float * 0.9, j as float * 0.9, 0.0),
                                           ball);
                bf.add(to_add);
                to_move.push(to_add);
            }
        }

        bf.update();

        // test deactivations followed by activations: this should not changes anything
        for e in to_move.mut_iter() {
            bf.deactivate(*e);
        }

        // nothing on pairs …
        assert_eq!(bf.num_interferences(), 0);
        // … because it has been transfered to spairs. NOTE: `spairs` is used for test only here,
        // there is no explicit way for the user to access it.

        for e in to_move.mut_iter() {
            bf.activate(*e, |_, _, _| { });
        }

        // test one deactivation followed by one activation: this should not changes anything
        for e in to_move.mut_iter() {
            bf.deactivate(*e);
            bf.activate(*e, |_, _, _| { });
        }

        for e in to_move.mut_iter() {
            let WithAABB(m, c) = **e;
            **e = WithAABB(Vec3::new(10.0, 10.0, 10.0) + m, c)
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

    #[test]
    fn test_dbvt_quadratic_collide() {
        type Shape = WithAABB<Vec3<float>, Ball<float>>;
        let dispatcher: NoIdDispatcher<Shape> = NoIdDispatcher;
        let mut bf     = DBVTBroadPhase::new(dispatcher, 0.2);
        let ball       = Ball::new(0.3);

        do 400.times {
            bf.add(@mut WithAABB(Vec3::new(0.0, 0.0, 0.0), ball))
        }

        bf.update();

        assert_eq!(bf.num_interferences(), (399 * (399 + 1)) / 2)
    }
}
