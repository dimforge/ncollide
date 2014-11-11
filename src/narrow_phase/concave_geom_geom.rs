use std::any::AnyRefExt;
use na::Inv;
use na;
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use bounding_volume::{HasAABB, BoundingVolume};
use partitioning::BoundingVolumeInterferencesCollector;
use broad_phase::Dispatcher;
use narrow_phase::{CollisionDetector, ShapeShapeDispatcher, ShapeShapeCollisionDetector,
                   DynamicCollisionDetector, CollisionDetectorFactory};
use shape::{Shape, ConcaveShape};
use geometry::Contact;
use math::{Scalar, Point};


/// Collision detector between a concave shape and another shape.
pub struct ConcaveShapeShape<N, P, V, M, I, G1, G2> {
    prediction:    N,
    sub_detectors: HashMap<uint, Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>, UintTWHash>,
    to_delete:     Vec<uint>,
    interferences: Vec<uint>
}

impl<N, P, V, M, I, G1, G2> ConcaveShapeShape<N, P, V, M, I, G1, G2> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new(prediction: N) -> ConcaveShapeShape<N, P, V, M, I, G1, G2> {
        ConcaveShapeShape {
            prediction:    prediction,
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:     Vec::new(),
            interferences: Vec::new()
        }
    }
}

impl<N, P, V, M, I, G1, G2> ConcaveShapeShape<N, P, V, M, I, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    fn do_update(&mut self,
                 dispatcher: &ShapeShapeDispatcher<N, P, V, M, I>,
                 m1:         &M,
                 g1:         &G1,
                 m2:         &M,
                 g2:         &G2,
                 swap:       bool) {
        // Find new collisions
        let ls_m2    = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2).loosened(self.prediction);
        let g2       = g2 as &Shape<N, P, V, M>;

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

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

impl<N, P, V, M, I, G1, G2> ShapeShapeCollisionDetector<N, P, V, M, I> for ConcaveShapeShape<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + ConcaveShape<N, P, V, M>,
          G2: 'static + Shape<N, P, V, M> {
    fn update(&mut self,
              dispatcher: &ShapeShapeDispatcher<N, P, V, M, I>,
              m1:         &M,
              g1:         &Shape<N, P, V, M>,
              m2:         &M,
              g2:         &Shape<N, P, V, M>) {
        self.do_update(dispatcher,
                       m1,
                       g1.downcast_ref::<G1>().expect("Invalid shape."),
                       m2,
                       g2.downcast_ref::<G2>().expect("Invalid shape."),
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

impl<N, P, V, M, I, G1, G2> DynamicCollisionDetector<N, P, V, M, I, G1, G2> for ConcaveShapeShape<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
}

/// Collision detector between a shape and a concave shape.
pub struct ShapeConcaveShape<N, P, V, M, I, G1, G2> {
    sub_detector: ConcaveShapeShape<N, P, V, M, I, G2, G1>
}

impl<N, P, V, M, I, G1, G2> ShapeConcaveShape<N, P, V, M, I, G1, G2> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new(prediction: N) -> ShapeConcaveShape<N, P, V, M, I, G1, G2> {
        ShapeConcaveShape {
            sub_detector: ConcaveShapeShape::new(prediction)
        }
    }
}

impl<N, P, V, M, I, G1, G2> ShapeShapeCollisionDetector<N, P, V, M, I> for ShapeConcaveShape<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + Shape<N, P, V, M>,
          G2: 'static + ConcaveShape<N, P, V, M> {
    fn update(&mut self,
              dispatcher: &ShapeShapeDispatcher<N, P, V, M, I>,
              m1:         &M,
              g1:         &Shape<N, P, V, M>,
              m2:         &M,
              g2:         &Shape<N, P, V, M>) {
        self.sub_detector.do_update(dispatcher,
                                    m2,
                                    g2.downcast_ref::<G2>().expect("Invalid shape."),
                                    m1,
                                    g1.downcast_ref::<G1>().expect("Invalid shape."),
                                    true);
    }

    fn num_colls(&self) -> uint {
        self.sub_detector.num_colls()
    }

    fn colls(&self, out: &mut Vec<Contact<N, P, V>>) {
        self.sub_detector.colls(out)
    }
}

impl<N, P, V, M, I, G1, G2> DynamicCollisionDetector<N, P, V, M, I, G1, G2> for ShapeConcaveShape<N, P, V, M, I, G1, G2>
    where N: Scalar,
          P:  Point<N, V>,
          M:  Inv + Mul<M, M>,
          G1: Shape<N, P, V, M>,
          G2: ConcaveShape<N, P, V, M> {
}

/*
 *
 * Custom factories
 *
 */
/// Structure implementing `CollisionDetectorFactory` in order to create a new `ConcaveShapeShape`
/// collision detector.
pub struct ConcaveShapeShapeFactory<N, P, V, M, G1, G2> {
    prediction: N
}

impl<N, P, V, M, G1, G2> ConcaveShapeShapeFactory<N, P, V, M, G1, G2> {
    /// Creates a `ConcaveShapeShapeFactory` with a given prediction length.
    pub fn new(prediction: N) -> ConcaveShapeShapeFactory<N, P, V, M, G1, G2> {
        ConcaveShapeShapeFactory {
            prediction: prediction
        }
    }
}

impl<N, P, V, M, I, G1, G2> CollisionDetectorFactory<N, P, V, M, I> for ConcaveShapeShapeFactory<N, P, V, M, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          V:  'static,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + ConcaveShape<N, P, V, M>,
          G2: 'static + Shape<N, P, V, M> {
    fn build(&self) -> Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send> {
        let res: ConcaveShapeShape<N, P, V, M, I, G1, G2> = ConcaveShapeShape::new(self.prediction.clone());
        box res as Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>
    }
}

/// Structure implementing `CollisionDetectorFactory` in order to create a new `ShapeConcaveShape`
/// collision detector.
pub struct ShapeConcaveShapeFactory<N, P, V, M, G1, G2> {
    prediction: N
}

impl<N, P, V, M, G1, G2> ShapeConcaveShapeFactory<N, P, V, M, G1, G2> {
    /// Creates a `ShapeConcaveShapeFactory` with a given prediction length.
    pub fn new(prediction: N) -> ShapeConcaveShapeFactory<N, P, V, M, G1, G2> {
        ShapeConcaveShapeFactory {
            prediction: prediction
        }
    }
}

impl<N, P, V, M, I, G1, G2> CollisionDetectorFactory<N, P, V, M, I> for ShapeConcaveShapeFactory<N, P, V, M, G1, G2>
    where N: Scalar,
          P:  'static + Point<N, V>,
          V:  'static,
          M:  'static + Inv + Mul<M, M>,
          I:  'static,
          G1: 'static + Shape<N, P, V, M>,
          G2: 'static + ConcaveShape<N, P, V, M> {
    fn build(&self) -> Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send> {
        let res: ShapeConcaveShape<N, P, V, M, I, G1, G2> = ShapeConcaveShape::new(self.prediction.clone());
        box res as Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>
    }
}
