use na;
use math::{Isometry, Point};
use utils::data::hash_map::HashMap;
use utils::data::hash::UintTWHash;
use geometry::bounding_volume::{self, BoundingVolume};
use geometry::partitioning::BoundingVolumeInterferencesCollector;
use geometry::shape::{CompositeShape, Shape};
use geometry::query::Proximity;
use narrow_phase::{ProximityAlgorithm, ProximityDetector, ProximityDispatcher};

/// Proximity detector between a concave shape and another shape.
pub struct CompositeShapeShapeProximityDetector<P: Point, M> {
    proximity: Proximity,
    sub_detectors: HashMap<usize, ProximityAlgorithm<P, M>, UintTWHash>,
    to_delete: Vec<usize>,
    interferences: Vec<usize>,
    intersecting_key: usize,
}

impl<P: Point, M> CompositeShapeShapeProximityDetector<P, M> {
    /// Creates a new proximity detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeShapeProximityDetector<P, M> {
        CompositeShapeShapeProximityDetector {
            proximity: Proximity::Disjoint,
            sub_detectors: HashMap::new_with_capacity(5, UintTWHash::new()),
            to_delete: Vec::new(),
            interferences: Vec::new(),
            intersecting_key: usize::max_value(),
        }
    }
}

impl<P: Point, M: Isometry<P>> CompositeShapeShapeProximityDetector<P, M> {
    fn do_update(
        &mut self,
        disp: &ProximityDispatcher<P, M>,
        m1: &M,
        g1: &CompositeShape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        margin: P::Real,
    ) {
        // Remove outdated sub detectors.
        for key in self.to_delete.iter() {
            let _ = self.sub_detectors.remove(key);
        }

        self.to_delete.clear();
        self.interferences.clear();

        // First, test if the previously intersecting shapes are still intersecting.
        if self.proximity == Proximity::Intersecting {
            let detector = self.sub_detectors.find_mut(&self.intersecting_key).unwrap();
            g1.map_transformed_part_at(self.intersecting_key, m1, &mut |_, m1, g1| {
                assert!(
                    detector.update(disp, m1, g1, m2, g2, margin),
                    "The shape was no longer valid."
                );
            });

            match detector.proximity() {
                Proximity::Intersecting => return, // Early return.
                Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                Proximity::Disjoint => {}
            }
        }

        self.proximity = Proximity::Disjoint;

        let m12 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &m12).loosened(margin);

        // Update all collisions
        for detector in self.sub_detectors.elements_mut().iter_mut() {
            let key = detector.key;

            if key == self.intersecting_key {
                // We already dealt with that one.
                continue;
            }

            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_transformed_part_at(key, m1, &mut |_, m1, g1| {
                    assert!(
                        detector.value.update(disp, m1, g1, m2, g2, margin),
                        "The shape was no longer valid."
                    );
                });

                match detector.value.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = detector.key;
                        return; // No need to search any further.
                    }
                    Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                self.to_delete.push(key);
            }
        }

        // Find new proximities.
        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for key in self.interferences.iter() {
            let detector = self.sub_detectors.find_or_insert_lazy(*key, || {
                let mut new_detector = None;

                g1.map_part_at(*key, &mut |_, _, g1| {
                    new_detector = disp.get_proximity_algorithm(g1, g2)
                });

                new_detector
            });

            if let Some(sub_detector) = detector {
                g1.map_transformed_part_at(*key, m1, &mut |_, m1, g1| {
                    let _ = sub_detector.update(disp, m1, g1, m2, g2, margin);
                });

                match sub_detector.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = *key;
                        return; // No need to search further.
                    }
                    Proximity::WithinMargin => self.proximity = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            }
        }

        // Totally disjoints.
        self.intersecting_key = usize::max_value()
    }
}

/// Proximity detector between a shape and a concave shape.
pub struct ShapeCompositeShapeProximityDetector<P: Point, M> {
    sub_detector: CompositeShapeShapeProximityDetector<P, M>,
}

impl<P: Point, M> ShapeCompositeShapeProximityDetector<P, M> {
    /// Creates a new collision detector between a shape and a concave shape.
    pub fn new() -> ShapeCompositeShapeProximityDetector<P, M> {
        ShapeCompositeShapeProximityDetector {
            sub_detector: CompositeShapeShapeProximityDetector::new(),
        }
    }
}

impl<P: Point, M: Isometry<P>> ProximityDetector<P, M>
    for CompositeShapeShapeProximityDetector<P, M>
{
    fn update(
        &mut self,
        disp: &ProximityDispatcher<P, M>,
        m1: &M,
        g1: &Shape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        margin: P::Real,
    ) -> bool {
        if let Some(cs1) = g1.as_composite_shape() {
            self.do_update(disp, m1, cs1, m2, g2, margin);

            true
        } else {
            false
        }
    }

    fn proximity(&self) -> Proximity {
        self.proximity
    }
}

impl<P: Point, M: Isometry<P>> ProximityDetector<P, M>
    for ShapeCompositeShapeProximityDetector<P, M>
{
    fn update(
        &mut self,
        disp: &ProximityDispatcher<P, M>,
        m1: &M,
        g1: &Shape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        margin: P::Real,
    ) -> bool {
        if let Some(cs2) = g2.as_composite_shape() {
            self.sub_detector.do_update(disp, m2, cs2, m1, g1, margin);

            true
        } else {
            false
        }
    }

    fn proximity(&self) -> Proximity {
        self.sub_detector.proximity()
    }
}
