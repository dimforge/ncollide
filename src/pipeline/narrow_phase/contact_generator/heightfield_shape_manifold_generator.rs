use crate::bounding_volume::{self, BoundingVolume};
use crate::math::Isometry;
use crate::pipeline::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use crate::query::{ContactManifold, ContactPrediction, ContactPreprocessor};
use crate::shape::{HeightField, Shape};
use crate::utils::DeterministicState;
use na::{self, RealField};
use std::collections::{hash_map::Entry, HashMap};

/// Collision detector between an heightfield and another shape.
pub struct HeightFieldShapeManifoldGenerator<N: RealField> {
    sub_detectors: HashMap<usize, (ContactAlgorithm<N>, usize), DeterministicState>,
    flip: bool,
    timestamp: usize,
}

impl<N: RealField> HeightFieldShapeManifoldGenerator<N> {
    /// Creates a new collision detector between an heightfield and another shape.
    pub fn new(flip: bool) -> HeightFieldShapeManifoldGenerator<N> {
        HeightFieldShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            flip,
            timestamp: 0,
        }
    }

    fn do_update(
        &mut self,
        dispatcher: &dyn ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &HeightField<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        g2: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) {
        self.timestamp += 1;

        // Find new collisions
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction.linear());

        g1.map_elements_in_local_aabb(&ls_aabb2, &mut |i, elt1, part_proc1| match self
            .sub_detectors
            .entry(i)
        {
            Entry::Occupied(mut entry) => {
                let ok = if self.flip {
                    entry.get_mut().0.generate_contacts(
                        dispatcher,
                        m2,
                        g2,
                        proc2,
                        m1,
                        elt1,
                        Some(&(proc1, part_proc1)),
                        prediction,
                        manifold,
                    )
                } else {
                    entry.get_mut().0.generate_contacts(
                        dispatcher,
                        m1,
                        elt1,
                        Some(&(proc1, part_proc1)),
                        m2,
                        g2,
                        proc2,
                        prediction,
                        manifold,
                    )
                };

                if ok {
                    entry.get_mut().1 = self.timestamp;
                }
            }
            Entry::Vacant(entry) => {
                let new_detector = if self.flip {
                    dispatcher.get_contact_algorithm(g2, elt1)
                } else {
                    dispatcher.get_contact_algorithm(elt1, g2)
                };

                if let Some(mut new_detector) = new_detector {
                    if self.flip {
                        let _ = new_detector.generate_contacts(
                            dispatcher,
                            m2,
                            g2,
                            proc2,
                            m1,
                            elt1,
                            Some(&(proc1, part_proc1)),
                            prediction,
                            manifold,
                        );
                    } else {
                        let _ = new_detector.generate_contacts(
                            dispatcher,
                            m1,
                            elt1,
                            Some(&(proc1, part_proc1)),
                            m2,
                            g2,
                            proc2,
                            prediction,
                            manifold,
                        );
                    }
                    let _ = entry.insert((new_detector, self.timestamp));
                }
            }
        });

        // Remove outdated entries.
        let timestamp = self.timestamp;
        self.sub_detectors
            .retain(|_, detector| detector.1 == timestamp);
    }
}

impl<N: RealField> ContactManifoldGenerator<N> for HeightFieldShapeManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        d: &dyn ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if !self.flip {
            if let Some(hf) = a.as_shape::<HeightField<N>>() {
                self.do_update(d, ma, hf, proc1, mb, b, proc2, prediction, manifold);
                return true;
            }
        } else {
            if let Some(hf) = b.as_shape::<HeightField<N>>() {
                self.do_update(d, mb, hf, proc2, ma, a, proc1, prediction, manifold);
                return true;
            }
        }

        return false;
    }

    //    fn init_manifold(&self) -> ContactManifold<N> {
    //        let mut res = ContactManifold::new();
    //        res.set_tracking_mode(ContactTrackingMode::FeatureBased);
    //        res
    //    }
}
