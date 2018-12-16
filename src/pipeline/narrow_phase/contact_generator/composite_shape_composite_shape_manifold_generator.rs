use bounding_volume::{BoundingVolume, AABB};
use math::Isometry;
use na::Real;
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{visitors::AABBSetsInterferencesCollector, ContactManifold, ContactPrediction, ContactPreprocessor};
use shape::{CompositeShape, FeatureId, Shape};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeCompositeShapeManifoldGenerator<N> {
    sub_detectors: HashMap<(usize, usize), (ContactAlgorithm<N>, usize), DeterministicState>,
    interferences: Vec<(usize, usize)>,
    timestamp: usize
}

impl<N> CompositeShapeCompositeShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeCompositeShapeManifoldGenerator<N> {
        CompositeShapeCompositeShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            interferences: Vec::new(),
            timestamp: 0
        }
    }
}

impl<N: Real> CompositeShapeCompositeShapeManifoldGenerator<N> {
    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        g2: &CompositeShape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    )
    {
        self.timestamp += 1;

        // Find new collisions
        let ls_m2 = m1.inverse() * m2;
        // For transforming AABBs from g2 in the local space of g1.
        let ls_m2_abs_rot = ls_m2.rotation.to_rotation_matrix().matrix().abs();

        {
            let mut visitor = AABBSetsInterferencesCollector::new(
                prediction.linear(),
                &ls_m2,
                &ls_m2_abs_rot,
                &mut self.interferences,
            );
            g1.bvh().visit_bvtt(g2.bvh(), &mut visitor);
        }

        for id in self.interferences.drain(..) {
            match self.sub_detectors.entry(id) {
                Entry::Occupied(mut entry) => {
                    entry.get_mut().1 = self.timestamp;
                }
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(id.0, &Isometry::identity(), &mut |_, g1| {
                        g2.map_part_at(id.1, &Isometry::identity(), &mut |_, g2| {
                            new_detector = dispatcher.get_contact_algorithm(g1, g2)
                        });
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
                false
            } else {
                let mut keep = false;
                g1.map_part_and_preprocessor_at(key.0, m1, prediction, &mut |m1, g1, proc1| {
                    g2.map_part_and_preprocessor_at(key.1, m2, prediction, &mut |m2, g2, proc2| {
                        // FIXME: change the update functions.
                        keep = detector.0.generate_contacts(
                            dispatcher, m1, g1, Some(proc1), m2, g2, Some(proc2), prediction, id_alloc,
                            manifold
                        );
                    });
                });

                keep
            }
        });
    }
}

impl<N: Real> ContactManifoldGenerator<N> for CompositeShapeCompositeShapeManifoldGenerator<N> {
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
        if let (Some(csa), Some(csb)) = (a.as_composite_shape(), b.as_composite_shape()) {
            self.do_update(
                d, ma, csa, proc1, mb, csb, proc2, prediction, id_alloc, manifold,
            );
            true
        } else {
            false
        }
    }
}
