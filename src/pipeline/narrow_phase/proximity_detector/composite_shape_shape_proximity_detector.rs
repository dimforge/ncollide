use crate::bounding_volume::{self, BoundingVolume};
use crate::math::Isometry;
use na::{self, RealField};
use crate::pipeline::narrow_phase::{ProximityAlgorithm, ProximityDetector, ProximityDispatcher};
use crate::query::{visitors::BoundingVolumeInterferencesCollector, Proximity};
use crate::shape::{CompositeShape, Shape};
use std::collections::{hash_map::Entry, HashMap};
use crate::utils::DeterministicState;

/// Proximity detector between a concave shape and another shape.
pub struct CompositeShapeShapeProximityDetector<N> {
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
            sub_detectors: HashMap::with_hasher(DeterministicState),
            to_delete: Vec::new(),
            interferences: Vec::new(),
            intersecting_key: usize::max_value(),
            flip,
        }
    }
}

impl<N: RealField> CompositeShapeShapeProximityDetector<N> {
    fn do_update(
        &mut self,
        dispatcher: &dyn ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &dyn CompositeShape<N>,
        m2: &Isometry<N>,
        g2: &dyn Shape<N>,
        margin: N,
        flip: bool)
        -> Option<Proximity>
    {
        let mut result = Proximity::Disjoint;

        // Remove outdated sub detectors.
        for key in self.to_delete.iter() {
            let _ = self.sub_detectors.remove(key);
        }

        self.to_delete.clear();
        self.interferences.clear();

        // First, test if the previously intersecting shapes are still intersecting.
        if self.intersecting_key != usize::max_value() {
            let detector = self.sub_detectors.get_mut(&self.intersecting_key).unwrap();
            let mut prox = None;
            g1.map_part_at(self.intersecting_key, m1, &mut |m1, g1| {
                prox = detector.update(dispatcher, m1, g1, m2, g2, margin)
            });

            match prox? {
                Proximity::Intersecting => return Some(Proximity::Intersecting), // Early return.
                Proximity::WithinMargin => result = Proximity::WithinMargin,
                Proximity::Disjoint => {}
            }
        }

        let m12 = m1.inverse() * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &m12).loosened(margin);

        // Update all collisions
        for detector in &mut self.sub_detectors {
            let key = *detector.0;

            if key == self.intersecting_key {
                // We already dealt with that one.
                continue;
            }

            if ls_aabb2.intersects(&g1.aabb_at(key)) {
                let mut prox = None;

                g1.map_part_at(key, m1, &mut |m1, g1| {
                    prox = detector.1.update(dispatcher, m1, g1, m2, g2, margin)
                });

                match prox? {
                    Proximity::Intersecting => {
                        self.intersecting_key = *detector.0;
                        return Some(Proximity::Intersecting); // No need to search any further.
                    }
                    Proximity::WithinMargin => result = Proximity::WithinMargin,
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

                    g1.map_part_at(*key, &Isometry::identity(), &mut |_, g1| {
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
                let mut prox = None;

                g1.map_part_at(*key, m1, &mut |m1, g1| {
                    if flip {
                        prox = sub_detector.update(dispatcher, m2, g2, m1, g1, margin);
                    } else {
                        prox = sub_detector.update(dispatcher, m1, g1, m2, g2, margin);
                    }
                });

                match prox? {
                    Proximity::Intersecting => {
                        self.intersecting_key = *key;
                        return Some(Proximity::Intersecting); // No need to search further.
                    }
                    Proximity::WithinMargin => result = Proximity::WithinMargin,
                    Proximity::Disjoint => {}
                }
            }
        }

        // Disjoints or within margin.
        self.intersecting_key = usize::max_value();
        Some(result)
    }
}

impl<N: RealField> ProximityDetector<N> for CompositeShapeShapeProximityDetector<N> {
    fn update(
        &mut self,
        dispatcher: &dyn ProximityDispatcher<N>,
        m1: &Isometry<N>,
        g1: &dyn Shape<N>,
        m2: &Isometry<N>,
        g2: &dyn Shape<N>,
        margin: N,
    ) -> Option<Proximity> {
        if !self.flip {
            let cs = g1.as_composite_shape()?;
            self.do_update(dispatcher, m1, cs, m2, g2, margin, false)
        } else {
            let cs = g2.as_composite_shape()?;
            self.do_update(dispatcher, m2, cs, m1, g1, margin, true)
        }
    }
}
