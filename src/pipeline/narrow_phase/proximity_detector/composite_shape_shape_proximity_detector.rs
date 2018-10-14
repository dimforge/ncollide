use bounding_volume::{self, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use pipeline::narrow_phase::{ProximityAlgorithm, ProximityDetector, ProximityDispatcher};
use query::{visitors::BoundingVolumeInterferencesCollector, Proximity};
use shape::{CompositeShape, Shape};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;

/// Proximity detector between a concave shape and another shape.
pub struct CompositeShapeShapeProximityDetector<N> {
    proximity: Proximity,
    sub_detectors: HashMap<usize, ProximityAlgorithm<N>, DeterministicState>,
    to_delete: Vec<usize>,
    interferences: Vec<usize>,
    intersecting_key: usize,
    flip: bool,
}

impl<N> CompositeShapeShapeProximityDetector<N> {
    /// Creates a new proximity detector between a concave shape and another shape.
    pub fn new(flip: bool) -> CompositeShapeShapeProximityDetector<N> {
        CompositeShapeShapeProximityDetector {
            proximity: Proximity::Disjoint,
            sub_detectors: HashMap::with_hasher(DeterministicState),
            to_delete: Vec::new(),
            interferences: Vec::new(),
            intersecting_key: usize::max_value(),
            flip,
        }
    }
}

impl<N: Real> CompositeShapeShapeProximityDetector<N> {
    fn do_update(
        &mut self,
        dispatcher: &ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        margin: N,
        flip: bool,
    ) {
        // Remove outdated sub detectors.
        for key in self.to_delete.iter() {
            let _ = self.sub_detectors.remove(key);
        }

        self.to_delete.clear();
        self.interferences.clear();

        // First, test if the previously intersecting shapes are still intersecting.
        if self.proximity == Proximity::Intersecting {
            let detector = self.sub_detectors.get_mut(&self.intersecting_key).unwrap();
            g1.map_transformed_part_at(self.intersecting_key, m1, &mut |_, m1, g1| {
                assert!(
                    detector.update(dispatcher, m1, g1, m2, g2, margin),
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
        for detector in &mut self.sub_detectors {
            let key = *detector.0;

            if key == self.intersecting_key {
                // We already dealt with that one.
                continue;
            }

            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                g1.map_transformed_part_at(key, m1, &mut |_, m1, g1| {
                    assert!(
                        detector.1.update(dispatcher, m1, g1, m2, g2, margin),
                        "The shape was no longer valid."
                    );
                });

                match detector.1.proximity() {
                    Proximity::Intersecting => {
                        self.proximity = Proximity::Intersecting;
                        self.intersecting_key = *detector.0;
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
            g1.bvh().visit(&mut visitor);
        }

        for key in &self.interferences {
            let entry = self.sub_detectors.entry(*key);
            let detector = match entry {
                Entry::Occupied(entry) => Some(entry.into_mut()),
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(*key, &mut |_, _, g1| {
                        if flip {
                            new_detector = dispatcher.get_proximity_algorithm(g2, g1)
                        } else {
                            new_detector = dispatcher.get_proximity_algorithm(g1, g2)
                        }
                    });

                    if let Some(new_detector) = new_detector {
                        Some(entry.insert(new_detector))
                    } else {
                        None
                    }
                }
            };

            if let Some(sub_detector) = detector {
                g1.map_transformed_part_at(*key, m1, &mut |_, m1, g1| {
                    if flip {
                        let _ = sub_detector.update(dispatcher, m2, g2, m1, g1, margin);
                    } else {
                        let _ = sub_detector.update(dispatcher, m1, g1, m2, g2, margin);
                    }
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

impl<N: Real> ProximityDetector<N> for CompositeShapeShapeProximityDetector<N> {
    fn update(
        &mut self,
        dispatcher: &ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &Shape<N>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        margin: N,
    ) -> bool {
        if let Some(cs1) = g1.as_composite_shape() {
            let flip = self.flip;
            self.do_update(dispatcher, m1, cs1, m2, g2, margin, flip);

            true
        } else {
            false
        }
    }

    fn proximity(&self) -> Proximity {
        self.proximity
    }
}
