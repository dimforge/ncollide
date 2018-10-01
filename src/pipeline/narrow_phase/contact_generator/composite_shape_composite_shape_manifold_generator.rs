use bounding_volume::{self, AABB, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{ContactManifold, ContactPrediction, visitors::AABBSetsInterferencesCollector};
use shape::{CompositeShape, Shape};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;


/// Collision detector between a concave shape and another shape.
pub struct CompositeShapeCompositeShapeManifoldGenerator<N> {
    sub_detectors: HashMap<(usize, usize), ContactAlgorithm<N>, DeterministicState>,
    to_delete: Vec<(usize, usize)>,
    interferences: Vec<(usize, usize)>,
}

impl<N> CompositeShapeCompositeShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CompositeShapeCompositeShapeManifoldGenerator<N> {
        CompositeShapeCompositeShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            to_delete: Vec::new(),
            interferences: Vec::new(),
        }
    }
}

impl<N: Real> CompositeShapeCompositeShapeManifoldGenerator<N> {
    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        id1: usize,
        m1: &Isometry<N>,
        g1: &CompositeShape<N>,
        id2: usize,
        m2: &Isometry<N>,
        g2: &CompositeShape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) {
        // Find new collisions
        let ls_m2 = m1.inverse() * m2;
        // For transforming AABBs from g2 in the local space of g1.
        let ls_m2_abs_rot = ls_m2.rotation.to_rotation_matrix().matrix().abs();

        {
            let mut visitor =
                AABBSetsInterferencesCollector::new(&ls_m2, &ls_m2_abs_rot, &mut self.interferences);
            g1.bvh().visit_bvtt(g2.bvh(), &mut visitor);
        }

        for id in self.interferences.drain(..) {
            match self.sub_detectors.entry(id) {
                Entry::Occupied(_) => {}
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    g1.map_part_at(id.0, &mut |_, _, g1| {
                        g2.map_part_at(id.1, &mut |_, _, g2| {
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
            let ls_aabb2 = AABB::from_half_extents(ls_m2 * aabb2.center(), ls_m2_abs_rot * aabb2.half_extents());

            if ls_aabb2.intersects(&aabb1) {
                g1.map_transformed_part_at(key.0, m1, &mut |sub_id1, m1, g1| {
                    g2.map_transformed_part_at(key.1, m2, &mut |sub_id2, m2, g2| {
                        assert!(
                            detector.update(
                                dispatcher,
                                id1 + sub_id1,
                                m1,
                                g1,
                                id2 + sub_id2,
                                m2,
                                g2,
                                prediction,
                                id_alloc,
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

impl<N: Real> ContactManifoldGenerator<N>
for CompositeShapeCompositeShapeManifoldGenerator<N> {
    fn update(
        &mut self,
        d: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(csa), Some(csb)) = (a.as_composite_shape(), b.as_composite_shape()) {
            self.do_update(d, ida, ma, csa, idb, mb, csb, prediction, id_alloc);
            true
        } else {
            false
        }
    }

    fn num_contacts(&self) -> usize {
        let mut res = 0;

        for detector in &self.sub_detectors {
            res = res + detector.1.num_contacts()
        }

        res
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        for detector in &self.sub_detectors {
            detector.1.contacts(out);
        }
    }
}
