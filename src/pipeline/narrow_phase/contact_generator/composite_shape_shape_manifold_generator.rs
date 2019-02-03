use crate::bounding_volume::{self, BoundingVolume};
use crate::math::Isometry;
use na::{self, Real};
use crate::pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use crate::query::{visitors::BoundingVolumeInterferencesCollector, ContactManifold, ContactPrediction, ContactPreprocessor, ContactTrackingMode};
use crate::shape::{CompositeShape, Shape};
use std::collections::{hash_map::Entry, HashMap};
use crate::utils::DeterministicState;
use crate::utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeShapeManifoldGenerator<N: Real> {
    sub_detectors: HashMap<usize, (ContactAlgorithm<N>, usize), DeterministicState>,
    interferences: Vec<usize>,
    flip: bool,
    timestamp: usize
}

impl<N: Real> CompositeShapeShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new(flip: bool) -> CompositeShapeShapeManifoldGenerator<N> {
        CompositeShapeShapeManifoldGenerator {
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
        g1: &CompositeShape<N>,
        _proc1: Option<&ContactPreprocessor<N>>,
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
        
        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvh().visit(&mut visitor);
        }
        
        for i in self.interferences.drain(..) {
            match self.sub_detectors.entry(i) {
                Entry::Occupied(mut entry) => {
                    entry.get_mut().1 = self.timestamp
                }
                Entry::Vacant(entry) => {
                    let mut new_detector = None;
        
                    g1.map_part_at(i, &Isometry::identity(), &mut |_, g1| {
                        if flip {
                            new_detector = dispatcher.get_contact_algorithm(g2, g1)
                        } else {
                            new_detector = dispatcher.get_contact_algorithm(g1, g2)
                        }
                    });
        
                    if let Some(new_detector) = new_detector {
                        let _ = entry.insert((new_detector, self.timestamp));
                    }
                }
            }
        }
        
        // Update all collisions
        let timestamp = self.timestamp;

        self.sub_detectors.retain(|key, detector| {
            if detector.1 != timestamp {
                // FIXME: ask the detector if it wants to be removed or not
                false
            } else {
                let mut keep = false;
                g1.map_part_and_preprocessor_at(*key, m1, prediction, &mut |m1, g1, proc1| {
                    keep = if flip {
                        detector.0.generate_contacts(
                            dispatcher,
                            m2,
                            g2,
                            proc2,
                            m1,
                            g1,
                            Some(proc1),
                            prediction,
                            id_alloc,
                            manifold
                        )
                    } else {
                        detector.0.generate_contacts(
                            dispatcher,
                            m1,
                            g1,
                            Some(proc1),
                            m2,
                            g2,
                            proc2,
                            prediction,
                            id_alloc,
                            manifold
                        )
                    }
                });

                keep
            }
        });
    }
}

impl<N: Real> ContactManifoldGenerator<N> for CompositeShapeShapeManifoldGenerator<N> {
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
            if let Some(cs) = a.as_composite_shape() {
                self.do_update(d, ma, cs, proc1, mb, b, proc2, prediction, id_alloc, manifold, false);
                return true;
            }
        } else {
            if let Some(cs) = b.as_composite_shape() {
                self.do_update(d, mb, cs, proc2, ma, a, proc1, prediction, id_alloc, manifold, true);
                return true;
            }
        }
        
        return false;
    }

    fn init_manifold(&self) -> ContactManifold<N> {
        let mut res = ContactManifold::new();
        res.set_tracking_mode(ContactTrackingMode::FeatureBased);
        res
    }
}
