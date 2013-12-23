use nalgebra::na::{Translation, Inv, AlgebraicVecExt};
use nalgebra::na;
use util::hash_map::HashMap;
use util::hash::UintTWHash;
use bounding_volume::{BoundingVolume, HasAABB};
use broad::Dispatcher;
use narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector,
             DynamicCollisionDetector, CollisionDetectorFactory};
use contact::Contact;
use geom::{Geom, ConcaveGeom};

pub struct ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> {
    priv sub_detectors: HashMap<uint, ~GeomGeomCollisionDetector<N, LV, AV, M, II>, UintTWHash>,
    priv to_delete:     ~[uint],
    priv interferences: ~[uint]
}

impl<N, LV, AV, M, II, G1, G2> ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> {
    pub fn new() -> ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> {
        ConcaveGeomGeom {
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:     ~[],
            interferences: ~[]
        }
    }
}

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: ConcaveGeom<N, LV, M, II>,
     G2: Geom<N, LV, M, II>>
ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> {
    fn do_update(&mut self,
                 dispatcher: &GeomGeomDispatcher<N, LV, AV, M, II>,
                 m1:         &M,
                 g1:         &G1,
                 m2:         &M,
                 g2:         &G2,
                 swap:       bool) {
        // Find new collisions
        let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);
        let g2       = g2 as &Geom<N, LV, M, II>;

        g1.approx_interferences_with_aabb(&ls_aabb2, &mut self.interferences);

        for i in self.interferences.iter() {
            g1.map_part_at(*i, |_, g1| {
                let detector;
                
                if swap {
                    detector = dispatcher.dispatch(g2, g1);
                }
                else {
                    detector = dispatcher.dispatch(g1, g2);
                }

                self.sub_detectors.insert_or_replace(*i, detector, false);
            });
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().mut_iter() {
            let key = detector.key;
            if ls_aabb2.intersects(g1.aabb_at(key)) {
                g1.map_transformed_part_at(m1, key, |m1, g1| {
                    if swap {
                        detector.value.update(dispatcher, m2, g2, m1, g1);
                    }
                    else {
                        detector.value.update(dispatcher, m1, g1, m2, g2);
                    }
                });
            }
            else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Remove outdated sub detectors
        for i in self.to_delete.iter() {
            self.sub_detectors.remove(i);
        }

        self.to_delete.clear();
    }
}

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: 'static + ConcaveGeom<N, LV, M, II>,
     G2: 'static + Geom<N, LV, M, II>>
GeomGeomCollisionDetector<N, LV, AV, M, II> for ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> {
    fn update(&mut self,
              dispatcher: &GeomGeomDispatcher<N, LV, AV, M, II>,
              m1:         &M,
              g1:         &Geom<N, LV, M, II>,
              m2:         &M,
              g2:         &Geom<N, LV, M, II>) {
        self.do_update(dispatcher,
                       m1,
                       g1.as_ref::<G1>().expect("Invalid geometry."),
                       m2,
                       g2.as_ref::<G2>().expect("Invalid geometry."),
                       false);
    }

    fn num_colls(&self) -> uint {
        let mut res = 0;

        for detector in self.sub_detectors.elements().iter() {
            res = res + detector.value.num_colls()
        }

        res
    }

    fn colls(&self, out: &mut ~[Contact<N, LV>]) {
        for detector in self.sub_detectors.elements().iter() {
            detector.value.colls(out);
        }
    }
}

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: ConcaveGeom<N, LV, M, II>,
     G2: Geom<N, LV, M, II>>
DynamicCollisionDetector<N, LV, AV, M, II, G1, G2> for ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> { }

pub struct GeomConcaveGeom<N, LV, AV, M, II, G1, G2> {
    priv sub_detector: ConcaveGeomGeom<N, LV, AV, M, II, G2, G1>
}

impl<N, LV, AV, M, II, G1, G2> GeomConcaveGeom<N, LV, AV, M, II, G1, G2> {
    pub fn new() -> GeomConcaveGeom<N, LV, AV, M, II, G1, G2> {
        GeomConcaveGeom {
            sub_detector: ConcaveGeomGeom::new()
        }
    }
}

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: 'static + Geom<N, LV, M, II>,
     G2: 'static + ConcaveGeom<N, LV, M, II>>
GeomGeomCollisionDetector<N, LV, AV, M, II> for GeomConcaveGeom<N, LV, AV, M, II, G1, G2> {
    fn update(&mut self,
              dispatcher: &GeomGeomDispatcher<N, LV, AV, M, II>,
              m1:         &M,
              g1:         &Geom<N, LV, M, II>,
              m2:         &M,
              g2:         &Geom<N, LV, M, II>) {
        self.sub_detector.do_update(dispatcher,
                                    m2,
                                    g2.as_ref::<G2>().expect("Invalid geometry."),
                                    m1,
                                    g1.as_ref::<G1>().expect("Invalid geometry."),
                                    true);
    }

    fn num_colls(&self) -> uint {
        self.sub_detector.num_colls()
    }

    fn colls(&self, out: &mut ~[Contact<N, LV>]) {
        self.sub_detector.colls(out)
    }
}

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: Geom<N, LV, M, II>,
     G2: ConcaveGeom<N, LV, M, II>>
DynamicCollisionDetector<N, LV, AV, M, II, G1, G2> for GeomConcaveGeom<N, LV, AV, M, II, G1, G2> { }

/*
 *
 * Custom factories
 *
 */
pub struct ConcaveGeomGeomFactory<N, LV, AV, M, II, G1, G2>;

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: 'static + ConcaveGeom<N, LV, M, II>,
     G2: 'static + Geom<N, LV, M, II>>
CollisionDetectorFactory<N, LV, AV, M, II> for ConcaveGeomGeomFactory<N, LV, AV, M, II, G1, G2> {
    fn build(&self) -> ~GeomGeomCollisionDetector<N, LV, AV, M, II> {
        let res: ConcaveGeomGeom<N, LV, AV, M, II, G1, G2> = ConcaveGeomGeom::new();
        ~res as ~GeomGeomCollisionDetector<N, LV, AV, M, II>
    }
}

pub struct GeomConcaveGeomFactory<N, LV, AV, M, II, G1, G2>;

impl<N:  Freeze + Send + Algebraic + Primitive + Orderable,
     LV: Freeze + Send + AlgebraicVecExt<N> + Clone,
     AV: Freeze + Send,
     M:  Freeze + Send + Inv + Mul<M, M> + Translation<LV>,
     II: Freeze + Send,
     G1: 'static + Geom<N, LV, M, II>,
     G2: 'static + ConcaveGeom<N, LV, M, II>>
CollisionDetectorFactory<N, LV, AV, M, II> for GeomConcaveGeomFactory<N, LV, AV, M, II, G1, G2> {
    fn build(&self) -> ~GeomGeomCollisionDetector<N, LV, AV, M, II> {
        let res: GeomConcaveGeom<N, LV, AV, M, II, G1, G2> = GeomConcaveGeom::new();
        ~res as ~GeomGeomCollisionDetector<N, LV, AV, M, II>
    }
}
