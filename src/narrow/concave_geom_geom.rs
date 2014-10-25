use std::any::AnyRefExt;
use na::Inv;
use na;
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use bounding_volume::BoundingVolume;
use bounding_volume::HasAABB;
use broad::Dispatcher;
use narrow::{CollisionDetector, GeomGeomDispatcher, GeomGeomCollisionDetector,
             DynamicCollisionDetector, CollisionDetectorFactory, Contact};
use geom::{Geom, ConcaveGeom};
use math::{Scalar, Point};


/// Collision detector between a concave geometry and another geometry.
pub struct ConcaveGeomGeom<N, P, V, M, I, G1, G2> {
    prediction:    N,
    sub_detectors: HashMap<uint, Box<GeomGeomCollisionDetector<N, P, V, M, I> + Send>, UintTWHash>,
    to_delete:     Vec<uint>,
    interferences: Vec<uint>
}

impl<N, P, V, M, I, G1, G2> ConcaveGeomGeom<N, P, V, M, I, G1, G2> {
    /// Creates a new collision detector between a concave geometry and another geometry.
    pub fn new(prediction: N) -> ConcaveGeomGeom<N, P, V, M, I, G1, G2> {
        ConcaveGeomGeom {
            prediction:    prediction,
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:     Vec::new(),
            interferences: Vec::new()
        }
    }
}

impl<N, P, V, M, I, G1, G2> ConcaveGeomGeom<N, P, V, M, I, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: ConcaveGeom<N, P, V, M>,
          G2: Geom<N, P, V, M> {
    fn do_update(&mut self,
                 dispatcher: &GeomGeomDispatcher<N, P, V, M, I>,
                 m1:         &M,
                 g1:         &G1,
                 m2:         &M,
                 g2:         &G2,
                 swap:       bool) {
        // Find new collisions
        let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2).loosened(self.prediction);
        let g2       = g2 as &Geom<N, P, V, M>;

        g1.approx_interferences_with_aabb(&ls_aabb2, &mut self.interferences);

        for i in self.interferences.iter() {
            let detector = g1.map_part_at(*i, |_, g1| {
                if swap {
                    dispatcher.dispatch(g2, g1)
                }
                else {
                    dispatcher.dispatch(g1, g2)
                }
            });

            match detector {
                Some(detector) => {
                    let _ = self.sub_detectors.insert_or_replace(*i, detector, false);
                },
                None => { }
            }
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
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

impl<N, P, V, M, I, G1, G2> GeomGeomCollisionDetector<N, P, V, M, I> for ConcaveGeomGeom<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + ConcaveGeom<N, P, V, M>,
          G2: 'static + Geom<N, P, V, M> {
    fn update(&mut self,
              dispatcher: &GeomGeomDispatcher<N, P, V, M, I>,
              m1:         &M,
              g1:         &Geom<N, P, V, M>,
              m2:         &M,
              g2:         &Geom<N, P, V, M>) {
        self.do_update(dispatcher,
                       m1,
                       g1.downcast_ref::<G1>().expect("Invalid geometry."),
                       m2,
                       g2.downcast_ref::<G2>().expect("Invalid geometry."),
                       false);
    }

    fn num_colls(&self) -> uint {
        let mut res = 0;

        for detector in self.sub_detectors.elements().iter() {
            res = res + detector.value.num_colls()
        }

        res
    }

    fn colls(&self, out: &mut Vec<Contact<N, P, V>>) {
        for detector in self.sub_detectors.elements().iter() {
            detector.value.colls(out);
        }
    }
}

impl<N, P, V, M, I, G1, G2> DynamicCollisionDetector<N, P, V, M, I, G1, G2> for ConcaveGeomGeom<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: ConcaveGeom<N, P, V, M>,
          G2: Geom<N, P, V, M> {
}

/// Collision detector between a geometry and a concave geometry.
pub struct GeomConcaveGeom<N, P, V, M, I, G1, G2> {
    sub_detector: ConcaveGeomGeom<N, P, V, M, I, G2, G1>
}

impl<N, P, V, M, I, G1, G2> GeomConcaveGeom<N, P, V, M, I, G1, G2> {
    /// Creates a new collision detector between a geometry and a concave geometry.
    pub fn new(prediction: N) -> GeomConcaveGeom<N, P, V, M, I, G1, G2> {
        GeomConcaveGeom {
            sub_detector: ConcaveGeomGeom::new(prediction)
        }
    }
}

impl<N, P, V, M, I, G1, G2> GeomGeomCollisionDetector<N, P, V, M, I> for GeomConcaveGeom<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + Geom<N, P, V, M>,
          G2: 'static + ConcaveGeom<N, P, V, M> {
    fn update(&mut self,
              dispatcher: &GeomGeomDispatcher<N, P, V, M, I>,
              m1:         &M,
              g1:         &Geom<N, P, V, M>,
              m2:         &M,
              g2:         &Geom<N, P, V, M>) {
        self.sub_detector.do_update(dispatcher,
                                    m2,
                                    g2.downcast_ref::<G2>().expect("Invalid geometry."),
                                    m1,
                                    g1.downcast_ref::<G1>().expect("Invalid geometry."),
                                    true);
    }

    fn num_colls(&self) -> uint {
        self.sub_detector.num_colls()
    }

    fn colls(&self, out: &mut Vec<Contact<N, P, V>>) {
        self.sub_detector.colls(out)
    }
}

impl<N, P, V, M, I, G1, G2> DynamicCollisionDetector<N, P, V, M, I, G1, G2> for GeomConcaveGeom<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: Geom<N, P, V, M>,
          G2: ConcaveGeom<N, P, V, M> {
}

/*
 *
 * Custom factories
 *
 */
/// Structure implementing `CollisionDetectorFactory` in order to create a new `ConcaveGeomGeom`
/// collision detector.
pub struct ConcaveGeomGeomFactory<N, P, V, M, G1, G2> {
    prediction: N
}

impl<N, P, V, M, G1, G2> ConcaveGeomGeomFactory<N, P, V, M, G1, G2> {
    /// Creates a `ConcaveGeomGeomFactory` with a given prediction length.
    pub fn new(prediction: N) -> ConcaveGeomGeomFactory<N, P, V, M, G1, G2> {
        ConcaveGeomGeomFactory {
            prediction: prediction
        }
    }
}

impl<N, P, V, M, I, G1, G2> CollisionDetectorFactory<N, P, V, M, I> for ConcaveGeomGeomFactory<N, P, V, M, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          V:  'static,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + ConcaveGeom<N, P, V, M>,
          G2: 'static + Geom<N, P, V, M> {
    fn build(&self) -> Box<GeomGeomCollisionDetector<N, P, V, M, I> + Send> {
        let res: ConcaveGeomGeom<N, P, V, M, I, G1, G2> = ConcaveGeomGeom::new(self.prediction.clone());
        box res as Box<GeomGeomCollisionDetector<N, P, V, M, I> + Send>
    }
}

/// Structure implementing `CollisionDetectorFactory` in order to create a new `GeomConcaveGeom`
/// collision detector.
pub struct GeomConcaveGeomFactory<N, P, V, M, G1, G2> {
    prediction: N
}

impl<N, P, V, M, G1, G2> GeomConcaveGeomFactory<N, P, V, M, G1, G2> {
    /// Creates a `GeomConcaveGeomFactory` with a given prediction length.
    pub fn new(prediction: N) -> GeomConcaveGeomFactory<N, P, V, M, G1, G2> {
        GeomConcaveGeomFactory {
            prediction: prediction
        }
    }
}

impl<N, P, V, M, I, G1, G2> CollisionDetectorFactory<N, P, V, M, I> for GeomConcaveGeomFactory<N, P, V, M, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          V:  'static,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + Geom<N, P, V, M>,
          G2: 'static + ConcaveGeom<N, P, V, M> {
    fn build(&self) -> Box<GeomGeomCollisionDetector<N, P, V, M, I> + Send> {
        let res: GeomConcaveGeom<N, P, V, M, I, G1, G2> = GeomConcaveGeom::new(self.prediction.clone());
        box res as Box<GeomGeomCollisionDetector<N, P, V, M, I> + Send>
    }
}
