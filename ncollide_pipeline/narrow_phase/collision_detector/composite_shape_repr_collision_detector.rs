use na;
use na::Translate;
use math::{Point, Vector, Isometry};
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use entities::bounding_volume::{self, BoundingVolume};
use entities::partitioning::BoundingVolumeInterferencesCollector;
use entities::shape::CompositeShape;
use entities::inspection::Shape;
use queries::geometry::Contact;
use narrow_phase::{CollisionDetector, CollisionDispatcher, CollisionAlgorithm};


/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeShapeCollisionDetector<P: Point, M> {
    sub_detectors: HashMap<usize, CollisionAlgorithm<P, M>, UintTWHash>,
    to_delete:     Vec<usize>,
    interferences: Vec<usize>
}

impl<P: Point, M> CompositeShapeShapeCollisionDetector<P, M> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeShapeCollisionDetector<P, M> {
        CompositeShapeShapeCollisionDetector {
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:     Vec::new(),
            interferences: Vec::new()
        }
    }
}

impl<P, M> CompositeShapeShapeCollisionDetector<P, M>
    where P:  Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn do_update(&mut self,
                 dispatcher: &CollisionDispatcher<P, M>,
                 m1:         &M,
                 g1:         &CompositeShape<P, M>,
                 m2:         &M,
                 g2:         &Shape<P, M>,
                 prediction: <P::Vect as Vector>::Scalar,
                 swap:       bool) {
        // Find new collisions
        let ls_m2    = na::inverse(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction);

        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for i in self.interferences.iter() {
            let _= self.sub_detectors.find_or_insert_lazy(*i,
                || {
                    let mut new_detector = None;

                    g1.map_part_at(*i, &mut |_, g1| {
                        let r1 = g1.desc();
                        let r2 = g2.desc();

                        if swap {
                            new_detector = dispatcher.get_collision_algorithm(&r2, &r1)
                        }
                        else {
                            new_detector = dispatcher.get_collision_algorithm(&r1, &r2)
                        }
                    });

                    new_detector
                }
            );
        }

        self.interferences.clear();

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
            let key = detector.key;
            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_transformed_part_at(key, m1, &mut |m1, g1| {
                    if swap {
                        assert!(detector.value.update(dispatcher, m2, g2, m1, g1, prediction),
                                "Internal error: the shape was no longer valid.");
                    }
                    else {
                        assert!(detector.value.update(dispatcher, m1, g1, m2, g2, prediction),
                                "Internal error: the shape was no longer valid.");
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

/// Collision detector between a shape and a concave shape.
pub struct ShapeCompositeShapeCollisionDetector<P: Point, M> {
    sub_detector: CompositeShapeShapeCollisionDetector<P, M>
}

impl<P: Point, M> ShapeCompositeShapeCollisionDetector<P, M> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new() -> ShapeCompositeShapeCollisionDetector<P, M> {
        ShapeCompositeShapeCollisionDetector {
            sub_detector: CompositeShapeShapeCollisionDetector::new()
        }
    }
}

impl<P, M> CollisionDetector<P, M> for CompositeShapeShapeCollisionDetector<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn update(&mut self,
              d:  &CollisionDispatcher<P, M>,
              ma: &M,
              a:  &Shape<P, M>,
              mb: &M,
              b:  &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let Some(cs) = a.desc().as_composite_shape() {
            self.do_update(d, ma, cs, mb, b, prediction, false);

            true
        }
        else {
            false
        }
    }

    fn num_colls(&self) -> usize {
        let mut res = 0;

        for detector in self.sub_detectors.elements().iter() {
            res = res + detector.value.num_colls()
        }

        res
    }

    fn colls(&self, out: &mut Vec<Contact<P>>) {
        for detector in self.sub_detectors.elements().iter() {
            detector.value.colls(out);
        }
    }
}

impl<P, M> CollisionDetector<P, M> for ShapeCompositeShapeCollisionDetector<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn update(&mut self,
              d:  &CollisionDispatcher<P, M>,
              ma: &M,
              a:  &Shape<P, M>,
              mb: &M,
              b:  &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let Some(cs) = b.desc().as_composite_shape() {
            self.sub_detector.do_update(d, mb, cs, ma, a, prediction, true);

            true
        }
        else {
            false
        }
    }

    fn num_colls(&self) -> usize {
        self.sub_detector.num_colls()
    }

    fn colls(&self, out: &mut Vec<Contact<P>>) {
        self.sub_detector.colls(out)
    }
}
