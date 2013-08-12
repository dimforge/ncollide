use std::vec;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use bounding_volume::bounding_volume::BoundingVolume;
use bounding_volume::aabb::{AABB, HasAABB};
use broad::dispatcher::Dispatcher;
use broad::dbvt::DBVTLeaf;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;
use geom::compound::CompoundAABB;

/// Collision detector between a `CompoundAABB` and any other shape. This other shape can itself be
/// a `CompoundAABB` but this is discouraged: use the `CompoundCompound` collision detector istead.
pub struct CompoundAABBAny<N, V, M, S, D, SD> {
    priv dispatcher:    D,
    priv sub_detectors: ~[Option<SD>],
    priv interferences: ~[@mut DBVTLeaf<V, uint, AABB<N, V>>],
    priv updated:       ~[bool]
}

/// Collision detector between any shape and a `CompoundAABB`. This is the same as
/// `CompoundAABBAny` but with the shapes swaped (the compound comes second on the argument of
/// `update`.
pub struct AnyCompoundAABB<N, V, M, S, D, SD> {
    priv sub_detector: CompoundAABBAny<N, V, M, S, D, SD>
}

impl<N, V, M, S, D, SD> CompoundAABBAny<N, V, M, S, D, SD> {
    /// Creates a new CompoundAABBAny collision detector.
    ///
    /// # Arguments:
    ///     * `dispatcher` - the collision dispatcher to build the collision detectors between the
    ///     compound geometry shapes and the other shape.
    ///     * `g` - the compound geometry to be handled by the detector.
    pub fn new(dispatcher: D, g: &CompoundAABB<N, V, M, S>) -> CompoundAABBAny<N, V, M, S, D, SD> {
        let nshapes           = g.shapes().len();
        let mut sub_detectors = vec::with_capacity(nshapes);

        // we do this to avoid the need of the `Clone` bound on `SD`.
        do nshapes.times() {
            sub_detectors.push(None)
        }

        CompoundAABBAny {
            dispatcher:    dispatcher,
            sub_detectors: sub_detectors,
            interferences: vec::with_capacity(nshapes),
            updated:       vec::from_elem(nshapes, false)
        }
    }
}

impl<N, V, M, S, D, SD> AnyCompoundAABB<N, V, M, S, D, SD> {
    /// Creates a new AnyCompoundAABB collision detector.
    ///
    /// # Arguments:
    ///     * `dispatcher` - the collision dispatcher to build the collision detectors between the
    ///     compound geometry shapes and the other shape.
    ///     * `g` - the compound geometry to be handled by the detector.
    pub fn new(dispatcher: D, g: &CompoundAABB<N, V, M, S>) -> AnyCompoundAABB<N, V, M, S, D, SD> {
        AnyCompoundAABB {
            sub_detector: CompoundAABBAny::new(dispatcher, g)
        }
    }
}

impl<N:  NumCast + Ord,
     V:  'static + VectorSpace<N> + Norm<N> + Ord + Orderable + Clone,
     M:  Inv + Mul<M, M>,
     S:  HasAABB<N, V, M>,
     D:  Dispatcher<S, SD>,
     SD: CollisionDetector<N, V, M, S, S>>
CompoundAABBAny<N, V, M, S, D, SD> {
    fn do_update(&mut self, m1: &M, g1: &CompoundAABB<N, V, M, S>, m2: &M, g2: &S, swap: bool) {
        for u in self.updated.mut_iter() {
            *u = false
        }

        // Find new collisions
        let ls_m2    = m1.inverse().expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        g1.dbvt().interferences_with_bounding_volume(&ls_aabb2, &mut self.interferences);

        for i in self.interferences.iter() {
            let g1 = g1.shapes()[i.object].second_ref();

            if self.sub_detectors[i.object].is_none() && self.dispatcher.is_valid(g1, g2) {
                if swap {
                    self.sub_detectors[i.object] = Some(self.dispatcher.dispatch(g2, g1))
                }
                else {
                    self.sub_detectors[i.object] = Some(self.dispatcher.dispatch(g1, g2))
                }
            }

            self.updated[i.object] = true;
        }

        self.interferences.clear();

        // Update all collisions
        for (i, detector) in self.sub_detectors.mut_iter().enumerate() {
            match *detector {
                None            => { },
                Some(ref mut d) => {
                    if self.updated[i] || ls_aabb2.intersects(&g1.leaves()[i].bounding_volume) {
                        // no more collision: remove the collision detector
                        let s1 = g1.shapes();
                        let new_child_transform = m1 * *s1[i].first_ref();

                        if swap {
                            d.update(m2, g2, &new_child_transform, s1[i].second_ref());
                        }
                        else {
                            d.update(&new_child_transform, s1[i].second_ref(), m2, g2);
                        }

                        // mark as not outdated
                        self.updated[i] = true
                    }
                }
            }
        }

        // Remove outdated sub detectors
        for (i, alive) in self.updated.iter().enumerate() {
            if !*alive {
                self.sub_detectors[i] = None
            }
        }
    }
}

impl<N:  NumCast + Ord,
     V:  'static + VectorSpace<N> + Norm<N> + Ord + Orderable + Clone,
     M:  Inv + Mul<M, M>,
     S:  HasAABB<N, V, M>,
     D:  Dispatcher<S, SD>,
     SD: CollisionDetector<N, V, M, S, S>>
CollisionDetector<N, V, M, CompoundAABB<N, V, M, S>, S>
for CompoundAABBAny<N, V, M, S, D, SD> {
    #[inline]
    fn update(&mut self, m1: &M, g1: &CompoundAABB<N, V, M, S>, m2: &M, g2: &S) {
        self.do_update(m1, g1, m2, g2, false)
    }

    #[inline]
    fn num_coll(&self) -> uint {
        let mut total = 0;

        for cd in self.sub_detectors.iter() {
            match *cd {
                Some(ref d) => total = total + d.num_coll(),
                None        => { }
            }
        }

        total
    }

    #[inline]
    fn colls(&self, out: &mut ~[Contact<N,V>]) {
        for cd in self.sub_detectors.iter() {
            match *cd {
                Some(ref d) => d.colls(out),
                None        => { }
            }
        }
    }
}

impl<N:  NumCast + Ord,
     V:  'static + VectorSpace<N> + Norm<N> + Ord + Orderable + Clone,
     M:  Inv + Mul<M, M>,
     S:  HasAABB<N, V, M>,
     D:  Dispatcher<S, SD>,
     SD: CollisionDetector<N, V, M, S, S>>
CollisionDetector<N, V, M, S, CompoundAABB<N, V, M, S>>
for AnyCompoundAABB<N, V, M, S, D, SD> {
    #[inline]
    fn update(&mut self, m1: &M, g1: &S, m2: &M, g2: &CompoundAABB<N, V, M, S>) {
        self.sub_detector.do_update(m2, g2, m1, g1, true)
    }

    #[inline]
    fn num_coll(&self) -> uint {
        self.sub_detector.num_coll()
    }

    #[inline]
    fn colls(&self, out: &mut ~[Contact<N, V>]) {
        self.sub_detector.colls(out);
    }
}
