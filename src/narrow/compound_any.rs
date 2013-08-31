use std::vec;
use nalgebra::mat::{Translation, Inv};
use nalgebra::vec::AlgebraicVecExt;
use bounding_volume::bounding_volume::BoundingVolume;
use bounding_volume::aabb::{AABB, HasAABB};
use broad::dispatcher::Dispatcher;
use partitioning::dbvt::DBVTLeaf;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;
use geom::compound::CompoundAABB;
use partitioning::bvt_visitor::BoundingVolumeInterferencesCollector;

/// Collision detector between a `CompoundAABB` and any other shape. This other shape can itself be
/// a `CompoundAABB` but this is discouraged: use the `CompoundCompound` collision detector istead.
pub struct CompoundAABBAny<N, V, M, G, D, SD> {
    priv dispatcher:    D,
    priv sub_detectors: ~[Option<SD>],
    priv interferences: ~[@mut DBVTLeaf<V, uint, AABB<N, V>>],
    priv updated:       ~[bool]
}

/// Collision detector between any shape and a `CompoundAABB`. This is the same as
/// `CompoundAABBAny` but with the shapes swaped (the compound comes second on the argument of
/// `update`.
pub struct AnyCompoundAABB<N, V, M, G, D, SD> {
    priv sub_detector: CompoundAABBAny<N, V, M, G, D, SD>
}

impl<N, V, M, G, D, SD> CompoundAABBAny<N, V, M, G, D, SD> {
    /// Creates a new CompoundAABBAny collision detector.
    ///
    /// # Arguments:
    ///     * `dispatcher` - the collision dispatcher to build the collision detectors between the
    ///     compound geometry shapes and the other shape.
    ///     * `g` - the compound geometry to be handled by the detector.
    pub fn new(dispatcher: D, g: &CompoundAABB<N, V, M, G>) -> CompoundAABBAny<N, V, M, G, D, SD> {
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

impl<N, V, M, G, D, SD> AnyCompoundAABB<N, V, M, G, D, SD> {
    /// Creates a new AnyCompoundAABB collision detector.
    ///
    /// # Arguments:
    ///     * `dispatcher` - the collision dispatcher to build the collision detectors between the
    ///     compound geometry shapes and the other shape.
    ///     * `g` - the compound geometry to be handled by the detector.
    pub fn new(dispatcher: D, g: &CompoundAABB<N, V, M, G>) -> AnyCompoundAABB<N, V, M, G, D, SD> {
        AnyCompoundAABB {
            sub_detector: CompoundAABBAny::new(dispatcher, g)
        }
    }
}

impl<N:  Algebraic + Primitive + Orderable + ToStr,
     V:  'static + Clone + AlgebraicVecExt<N> + ToStr,
     M:  Inv + Mul<M, M>,
     G:  HasAABB<N, V, M>,
     D:  Dispatcher<G, SD>,
     SD: CollisionDetector<N, V, M, G, G>>
CompoundAABBAny<N, V, M, G, D, SD> {
    fn do_update(&mut self, m1: &M, g1: &CompoundAABB<N, V, M, G>, m2: &M, g2: &G, swap: bool) {
        for u in self.updated.mut_iter() {
            *u = false
        }

        // Find new collisions
        let ls_m2    = m1.inverse().expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.dbvt().visit(&mut visitor);
        }

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

impl<N:  Algebraic + Primitive + Orderable + ToStr,
     V:  'static + Clone + AlgebraicVecExt<N> + ToStr,
     M:  Inv + Mul<M, M> + Translation<V>,
     G:  HasAABB<N, V, M>,
     D:  Dispatcher<G, SD>,
     SD: CollisionDetector<N, V, M, G, G>>
CollisionDetector<N, V, M, CompoundAABB<N, V, M, G>, G>
for CompoundAABBAny<N, V, M, G, D, SD> {
    #[inline]
    fn update(&mut self, m1: &M, g1: &CompoundAABB<N, V, M, G>, m2: &M, g2: &G) {
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

    #[inline]
    fn toi(_:    Option<CompoundAABBAny<N, V, M, G, D, SD>>,
           m1:   &M,
           dir:  &V,
           dist: &N,
           g1:   &CompoundAABB<N, V, M, G>,
           m2:   &M,
           g2:   &G)
           -> Option<N> {
        let _self: Option<AnyCompoundAABB<N, V, M, G, D, SD>> = None;
        CollisionDetector::toi(_self, m2, &-dir, dist, g2, m1, g1)
    }
}

impl<N:  Algebraic + Primitive + Orderable + ToStr,
     V:  'static + AlgebraicVecExt<N> + Clone + ToStr,
     M:  Inv + Mul<M, M> + Translation<V>,
     G:  HasAABB<N, V, M>,
     D:  Dispatcher<G, SD>,
     SD: CollisionDetector<N, V, M, G, G>>
CollisionDetector<N, V, M, G, CompoundAABB<N, V, M, G>>
for AnyCompoundAABB<N, V, M, G, D, SD> {
    #[inline]
    fn update(&mut self, m1: &M, g1: &G, m2: &M, g2: &CompoundAABB<N, V, M, G>) {
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

    #[inline]
    fn toi(_:    Option<AnyCompoundAABB<N, V, M, G, D, SD>>,
           m1:   &M,
           dir:  &V,
           dist: &N,
           g1:   &G,
           m2:   &M,
           g2:   &CompoundAABB<N, V, M, G>)
           -> Option<N> {
        let inv_m2        = m2.inverse().expect("The transformation `m2` must be inversible.");
        let ls_m1_begin   = inv_m2 * *m1;
        let m1_end        = m1.translated(&(dir * *dist));
        let ls_m1_end     = inv_m2 * m1_end;
        let ls_aabb_begin = g1.aabb(&ls_m1_begin);
        let ls_aabb_end   = g1.aabb(&ls_m1_end);
        let ls_swept_aabb = ls_aabb_begin.merged(&ls_aabb_end);

        // FIXME: too bad we have to allocate here…
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[@mut DBVTLeaf<V, uint, AABB<N, V>>] = ~[];

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_swept_aabb, &mut interferences);
            g2.dbvt().visit(&mut visitor);
        }

        let mut min_toi: N = Bounded::max_value();

        for i in interferences.iter() {
            let child_m2 = m2 * *g2.shapes()[i.object].first_ref();
            let g2       = g2.shapes()[i.object].second_ref();

            match CollisionDetector::toi(None::<SD>, m1, dir, dist, g1, &child_m2, g2) {
                Some(toi) => min_toi = min_toi.min(&toi),
                None => { }
            }
        }

        if min_toi != Bounded::max_value() {
            Some(min_toi)
        }
        else {
            None
        }
    }
}
