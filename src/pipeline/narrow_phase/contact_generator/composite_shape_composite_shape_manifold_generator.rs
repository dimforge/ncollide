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
    sub_detectors: HashMap<(usize, usize), ContactAlgorithm<N>, DeterministicState>,
    interferences: Vec<(usize, usize)>,
}

impl<N> CompositeShapeCompositeShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeCompositeShapeManifoldGenerator<N> {
        CompositeShapeCompositeShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            interferences: Vec::new(),
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
                Entry::Occupied(_) => {}
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(id.0, &Isometry::identity(), &mut |_, g1| {
                        g2.map_part_at(id.1, &Isometry::identity(), &mut |_, g2| {
                            new_detector = dispatcher.get_contact_algorithm(g1, g2)
                        });
                    });

                    if let Some(new_detector) = new_detector {
                        let _ = entry.insert(new_detector);
                    }
                }
            }
        }

        // Update all collisions
        self.sub_detectors.retain(|key, detector| {
            let aabb1 = g1.aabb_at(key.0);
            let aabb2 = g2.aabb_at(key.1);
            let ls_aabb2 = AABB::from_half_extents(
                ls_m2 * aabb2.center(),
                ls_m2_abs_rot * aabb2.half_extents(),
            );

            if ls_aabb2.intersects(&aabb1) {
                g1.map_part_at(key.0, m1, &mut |m1, g1| {
                    g2.map_part_at(key.1, m2, &mut |m2, g2| {
                        // FIXME: change the update functions.
                        assert!(
                            detector.generate_contacts(
                                dispatcher, m1, g1, proc1, m2, g2, proc2, prediction, id_alloc,
                                manifold
                            ),
                            "Internal error: the shape was no longer valid."
                        );
                    });
                });

                true
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                false
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
