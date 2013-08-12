use std::ptr;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::translation::Translation;
use broad::dbvt::{DBVT, DBVTLeaf};
use util::hash::UintTWHash;
use util::hash_map::HashMap;
use util::pair::{Pair, PairTWHash};
use broad::dispatcher::Dispatcher;
use bounding_volume::bounding_volume::{HasBoundingVolume, LooseBoundingVolume};

/// Broad phase based on a Dynamic Bounding Volume Tree. It uses two separate trees: one for static
/// objects and which is never updated, and one for moving objects.
pub struct DBVTBroadPhase<N, V, B, BV, D, DV> {
    priv tree:       DBVT<V, @mut B, BV>,
    priv stree:      DBVT<V, @mut B, BV>,
    priv active:     ~[@mut DBVTLeaf<V, @mut B, BV>],
    priv rs2bv:      HashMap<uint, @mut DBVTLeaf<V, @mut B, BV>, UintTWHash>,
    priv pairs:      HashMap<Pair<DBVTLeaf<V, @mut B, BV>>, DV, PairTWHash>, // pair manager
    priv dispatcher: D,
    priv margin:     N,
    priv to_update:  ~[@mut DBVTLeaf<V, @mut B, BV>],
    priv update_off: uint // incremental pairs removal index
}

impl<N:  Clone + Ord,
     V:  'static + Sub<V, V> + Norm<N>,
     B:  'static + HasBoundingVolume<BV>,
     BV: 'static + LooseBoundingVolume<N> + Translation<V>,
     D:  Dispatcher<B, DV>,
     DV>
DBVTBroadPhase<N, V, B, BV, D, DV> {
    /// Creates a new broad phase based on a Dynamic Bounding Volume Tree.
    pub fn new(dispatcher: D, margin: N) -> DBVTBroadPhase<N, V, B, BV, D, DV> {
        DBVTBroadPhase {
            tree:       DBVT::new(),
            stree:      DBVT::new(),
            rs2bv:      HashMap::new(UintTWHash),
            pairs:      HashMap::new(PairTWHash),
            dispatcher: dispatcher,
            update_off: 0,
            active:     ~[],
            to_update:  ~[],
            margin:     margin
        }
    }

    /// The pair manager of this broad phase.
    pub fn pairs<'r>(&'r self) -> &'r HashMap<Pair<DBVTLeaf<V, @mut B, BV>>, DV, PairTWHash> {
        &'r self.pairs
    }

    /// The pair manager of this broad phase.
    pub fn pairs_mut<'r>(&'r mut self)
                         -> &'r mut HashMap<Pair<DBVTLeaf<V, @mut B, BV>>, DV, PairTWHash> {
        &'r mut self.pairs
    }

    /// Adds an element to this broad phase.
    pub fn add(&mut self, rb: @mut B) {
        let leaf = @mut DBVTLeaf::new(rb.bounding_volume().loosened(self.margin.clone()), rb);

        self.to_update.push(leaf);
        self.update_updatable();

        self.active.push(leaf);
        self.rs2bv.insert(ptr::to_mut_unsafe_ptr(rb) as uint, leaf);
    }

    /// Removes an element from this broad phase.
    pub fn remove(&mut self, _: @mut B) {
        fail!("Not yet implemented.");
    }

    /// Marks and object as active or inactive. The bounding volume of an inactive object is never
    /// updated. Activating/deactivating an already active/inactive objecs leads to undefined
    /// behaviour.
    pub fn set_active(&mut self, _: @mut B, _: bool) {
        fail!("Not yet implemented.");
    }

    /// Updates the collision pairs based on the objects bounding volumes.
    pub fn update(&mut self) {
        // NOTE: be careful not to add the same object twice!

        /*
         * Remove all outdated nodes
         */
        for a in self.active.iter() {
            let mut new_bv = a.object.bounding_volume();

            if !a.bounding_volume.contains(&new_bv) {
                // need an update!
                new_bv.loosen(self.margin.clone());
                a.bounding_volume = new_bv;
                self.tree.remove(*a);
                self.to_update.push(*a);
            }
        }

        self.update_updatable();
    }

    fn update_updatable(&mut self) {
        /*
         * Re-insert outdated nodes one by one and collect interferences at the same time.
         */
        let mut interferences = ~[];


        let mut new_colls = 0u;

        for u in self.to_update.iter() {
            self.tree.interferences_with_leaf(*u, &mut interferences);

            // dispatch
            for o in interferences.iter() {
                if self.dispatcher.is_valid(u.object, o.object) {
                    if u.bounding_volume.intersects(&o.bounding_volume) {
                        self.pairs.find_or_insert_lazy(
                            Pair::new(*u, *o),
                            || self.dispatcher.dispatch(u.object, o.object)
                            );

                        new_colls = new_colls + 1;
                    }
                }
            }

            interferences.clear();
            self.tree.insert(*u);
        }

        // interferences against the static tree
        for u in self.to_update.iter() {
            self.stree.interferences_with_leaf(*u, &mut interferences);

            // dispatch
            for o in interferences.iter() {
                if self.dispatcher.is_valid(u.object, o.object) {
                    if u.bounding_volume.intersects(&o.bounding_volume) {
                        self.pairs.find_or_insert_lazy(
                            Pair::new(*u, *o),
                            || self.dispatcher.dispatch(u.object, o.object)
                            );

                        new_colls = new_colls + 1;
                    }
                }
            }

            interferences.clear();
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

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::vec::Vec3;
    use geom::ball::Ball;
    use bounding_volume::aabb::WithAABB;
    use broad::dispatcher::NoIdDispatcher;

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

        assert_eq!(bf.pairs().len(), 0)
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
            bf.pairs().len(),
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

        for e in to_move.mut_iter() {
            let WithAABB(m, c) = **e;
            **e = WithAABB(Vec3::new(10.0, 10.0, 10.0) + m, c)
        }

        bf.update();

        assert_eq!(
            bf.pairs().len(),
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

        assert_eq!(bf.pairs().len(), (399 * (399 + 1)) / 2)
    }
}
