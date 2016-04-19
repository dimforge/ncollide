use na;
use na::Translate;
use math::{Point, Vector, Isometry};
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use entities::bounding_volume::{self, BoundingVolume};
use entities::partitioning::BoundingVolumeInterferencesCollector;
use entities::shape::CompositeShape;
use entities::inspection::Repr;
use entities::inspection;
use queries::geometry::Proximity;
use narrow_phase::{ProximityDetector, ProximityDispatcher, ProximityAlgorithm};


/// Proximity detector between a concave shape and another shape.
pub struct CompositeShapeReprProximityDetector<P: Point, M> {
    proximity:        Proximity,
    sub_detectors:    HashMap<usize, ProximityAlgorithm<P, M>, UintTWHash>,
    to_delete:        Vec<usize>,
    interferences:    Vec<usize>,
    intersecting_key: usize
}

impl<P: Point, M> CompositeShapeReprProximityDetector<P, M> {
    /// Creates a new proximity detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeReprProximityDetector<P, M> {
        CompositeShapeReprProximityDetector {
            proximity:        Proximity::Disjoint,
            sub_detectors:    HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete:        Vec::new(),
            interferences:    Vec::new(),
            intersecting_key: usize::max_value()
        }
    }
}

impl<P, M> CompositeShapeReprProximityDetector<P, M>
    where P:  Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn do_update(&mut self,
                 disp:   &ProximityDispatcher<P, M>,
                 m1:     &M,
                 g1:     &CompositeShape<P, M>,
                 m2:     &M,
                 g2:     &Repr<P, M>,
                 margin: <P::Vect as Vector>::Scalar) {
        // Remove outdated sub detectors.
        for key in self.to_delete.iter() {
            self.sub_detectors.remove(key);
        }

        self.to_delete.clear();
        self.interferences.clear();

        // First, test if the previously intersecting shapes are still intersecting.
        if self.proximity == Proximity::Intersecting {
            let detector = self.sub_detectors.find_mut(&self.intersecting_key).unwrap();
            g1.map_transformed_part_at(self.intersecting_key, m1, &mut |m1, g1| {
                assert!(detector.update(disp, m1, g1, m2, g2, margin), "The shape was no longer valid.");
            });

            match detector.proximity() {
                Proximity::Intersecting => return, // Early return.
                Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                Proximity::Disjoint => { }
            }
        }

        self.proximity = Proximity::Disjoint;

        let m12      = na::inverse(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = bounding_volume::aabb(g2, &m12).loosened(margin);

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
            let key = detector.key;

            if key == self.intersecting_key {
                // We already dealt with that one.
                continue;
            }

            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_transformed_part_at(key, m1, &mut |m1, g1| {
                    assert!(detector.value.update(disp, m1, g1, m2, g2, margin), "The shape was no longer valid.");
                });

                match detector.value.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = detector.key;
                        return; // No need to search any further.
                    },
                    Proximity::WithinMargin => {
                        self.proximity = Proximity::WithinMargin
                    },
                    Proximity::Disjoint => { }
                }
            }
            else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Find new proximities.
        {
            let mut visitor = BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for key in self.interferences.iter() {
            let detector = self.sub_detectors.find_or_insert_lazy(*key,
                || {
                    let mut new_detector = None;

                    g1.map_part_at(*key, &mut |_, g1| {
                        new_detector = disp.get_proximity_algorithm(&g1.repr(), &g2.repr())
                    });

                    new_detector
                }
            );

            if let Some(sub_detector) = detector {
                g1.map_transformed_part_at(*key, m1, &mut |m1, g1| {
                    sub_detector.update(disp, m1, g1, m2, g2, margin);
                });

                match sub_detector.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = *key;
                        return; // No need to search further.
                    },
                    Proximity::WithinMargin => {
                        self.proximity = Proximity::WithinMargin
                    },
                    Proximity::Disjoint => { }
                }
            }
        }

        // Totally disjoints.
        self.intersecting_key = usize::max_value()
    }
}

/// Proximity detector between a shape and a concave shape.
pub struct ReprCompositeShapeProximityDetector<P: Point, M> {
    sub_detector: CompositeShapeReprProximityDetector<P, M>
}

impl<P: Point, M> ReprCompositeShapeProximityDetector<P, M> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new() -> ReprCompositeShapeProximityDetector<P, M> {
        ReprCompositeShapeProximityDetector {
            sub_detector: CompositeShapeReprProximityDetector::new()
        }
    }
}

impl<P, M> ProximityDetector<P, M> for CompositeShapeReprProximityDetector<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn update(&mut self,
              disp:   &ProximityDispatcher<P, M>,
              m1: &M, g1: &Repr<P, M>,
              m2: &M, g2: &Repr<P, M>,
              margin: <P::Vect as Vector>::Scalar)
              -> bool {
        if let Some(cs1) = inspection::maybe_as_composite_shape(g1) {
            self.do_update(disp, m1, cs1, m2, g2, margin);

            true
        }
        else {
            false
        }
    }

    fn proximity(&self) -> Proximity {
        self.proximity
    }
}

impl<P, M> ProximityDetector<P, M> for ReprCompositeShapeProximityDetector<P, M>
    where P: Point,
          P::Vect: Translate<P>,
          M: Isometry<P, P::Vect> {
    fn update(&mut self,
              disp:  &ProximityDispatcher<P, M>,
              m1: &M, g1: &Repr<P, M>,
              m2: &M, g2: &Repr<P, M>,
              margin: <P::Vect as Vector>::Scalar)
              -> bool {
        if let Some(cs2) = inspection::maybe_as_composite_shape(g2) {
            self.sub_detector.do_update(disp, m2, cs2, m1, g1, margin);

            true
        }
        else {
            false
        }
    }

    fn proximity(&self) -> Proximity {
        self.sub_detector.proximity()
    }
}
