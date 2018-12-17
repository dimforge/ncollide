use bounding_volume::{self, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{visitors::BoundingVolumeInterferencesCollector, ContactManifold, ContactPrediction, ContactPreprocessor, ContactTrackingMode};
use shape::{CompositeShape, FeatureId, Shape, HeightField};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between an heightfield and another shape.
pub struct HeightFieldShapeManifoldGenerator<N: Real> {
    sub_detectors: HashMap<usize, (ContactAlgorithm<N>, usize), DeterministicState>,
    interferences: Vec<usize>,
    flip: bool,
    timestamp: usize
}

impl<N: Real> HeightFieldShapeManifoldGenerator<N> {
    /// Creates a new collision detector between an heightfield and another shape.
    pub fn new(flip: bool) -> HeightFieldShapeManifoldGenerator<N> {
        HeightFieldShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            interferences: Vec::new(),
            flip,
            timestamp: 0
        }
    }

    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &HeightField<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
        flip: bool,
    )
    {
        self.timestamp += 1;

        // Find new collisions
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction.linear());

        g1.map_elements_in_local_aabb(&ls_aabb2, &mut |i, elt1, proc1| {
            match self.sub_detectors.entry(i) {
                Entry::Occupied(mut entry) => {
                    let ok = if flip {
                        entry.get_mut().0.generate_contacts(
                            dispatcher,
                            m2,
                            g2,
                            proc2,
                            m1,
                            elt1,
                            Some(proc1),
                            prediction,
                            id_alloc,
                            manifold
                        )
                    } else {
                        entry.get_mut().0.generate_contacts(
                            dispatcher,
                            m1,
                            elt1,
                            Some(proc1),
                            m2,
                            g2,
                            proc2,
                            prediction,
                            id_alloc,
                            manifold
                        )
                    };

                    if ok {
                        entry.get_mut().1 = self.timestamp;
                    }
                }
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    if flip {
                        new_detector = dispatcher.get_contact_algorithm(g2, elt1)
                    } else {
                        new_detector = dispatcher.get_contact_algorithm(elt1, g2)
                    }

                    if let Some(mut new_detector) = new_detector {
                        if flip {
                            let _ = new_detector.generate_contacts(
                                dispatcher,
                                m2,
                                g2,
                                proc2,
                                m1,
                                elt1,
                                Some(proc1),
                                prediction,
                                id_alloc,
                                manifold
                            );
                        } else {
                            let _ = new_detector.generate_contacts(
                                dispatcher,
                                m1,
                                elt1,
                                Some(proc1),
                                m2,
                                g2,
                                proc2,
                                prediction,
                                id_alloc,
                                manifold
                            );
                        }
                        let _ = entry.insert((new_detector, self.timestamp));
                    }
                }
            }
        });

        
        // Remove outdated entries.
        let timestamp = self.timestamp;
        self.sub_detectors.retain(|_, detector| {
            detector.1 == timestamp
        });
    }
}

impl<N: Real> ContactManifoldGenerator<N> for HeightFieldShapeManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        d: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if !self.flip {
            if let Some(hf) = a.as_shape::<HeightField<N>>() {
                self.do_update(d, ma, hf, proc1, mb, b, proc2, prediction, id_alloc, manifold, false);
                return true;
            }
        } else {
            if let Some(hf) = b.as_shape::<HeightField<N>>() {
                self.do_update(d, mb, hf, proc2, ma, a, proc1, prediction, id_alloc, manifold, true);
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
