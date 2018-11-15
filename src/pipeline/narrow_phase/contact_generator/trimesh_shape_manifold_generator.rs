use bounding_volume::{self, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use partitioning::BVH;
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{
    visitors::BoundingVolumeInterferencesCollector, ContactManifold, ContactPrediction,
    ContactTrackingMode,
};
use shape::{CompositeShape, FeatureId, Shape, TriMesh, Triangle};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct TriMeshShapeManifoldGenerator<N: Real> {
    sub_detectors: HashMap<usize, ContactAlgorithm<N>, DeterministicState>,
    interferences: Vec<usize>,
    flip: bool,
}

impl<N: Real> TriMeshShapeManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new(flip: bool) -> TriMeshShapeManifoldGenerator<N> {
        TriMeshShapeManifoldGenerator {
            sub_detectors: HashMap::with_hasher(DeterministicState),
            interferences: Vec::new(),
            flip,
        }
    }

    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &TriMesh<N>,
        fmap1: Option<&Fn(FeatureId) -> FeatureId>,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        fmap2: Option<&Fn(FeatureId) -> FeatureId>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) {
        // Find new collisions
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = bounding_volume::aabb(g2, &ls_m2).loosened(prediction.linear());

        {
            let mut visitor =
                BoundingVolumeInterferencesCollector::new(&ls_aabb2, &mut self.interferences);
            g1.bvt().visit(&mut visitor);
        }

        for i in self.interferences.drain(..) {
            match self.sub_detectors.entry(i) {
                Entry::Occupied(_) => {}
                Entry::Vacant(entry) => {
                    let mut new_detector = None;

                    let tri = g1.triangle_at(i);
                    if self.flip {
                        new_detector = dispatcher.get_contact_algorithm(g2, &tri)
                    } else {
                        new_detector = dispatcher.get_contact_algorithm(&tri, g2)
                    }

                    if let Some(new_detector) = new_detector {
                        let _ = entry.insert(new_detector);
                    }
                }
            }
        }

        // Update all collisions
        let flip = self.flip;
        self.sub_detectors.retain(|key, detector| {
            let face = &g1.faces()[*key];

            if ls_aabb2.intersects(&g1.aabb_at(*key)) {
                let tri = Triangle::new(
                    g1.points()[face.indices.x],
                    g1.points()[face.indices.y],
                    g1.points()[face.indices.z],
                );

                let remap_feature = |f| match f {
                    FeatureId::Vertex(i) => FeatureId::Vertex(face.indices[i]),
                    FeatureId::Edge(i) => FeatureId::Edge(face.edges[i]),
                    FeatureId::Face(i) => {
                        if i == 0 {
                            FeatureId::Face(*key)
                        } else {
                            FeatureId::Face(*key + g1.faces().len())
                        }
                    }
                    FeatureId::Unknown => FeatureId::Unknown,
                };

                if flip {
                    assert!(
                        detector.generate_contacts(
                            dispatcher,
                            m2,
                            g2,
                            fmap2,
                            m1,
                            &tri,
                            Some(&remap_feature),
                            prediction,
                            id_alloc,
                            manifold
                        ),
                        "Internal error: the shape was no longer valid."
                    );
                } else {
                    assert!(
                        detector.generate_contacts(
                            dispatcher,
                            m1,
                            &tri,
                            Some(&remap_feature),
                            m2,
                            g2,
                            fmap2,
                            prediction,
                            id_alloc,
                            manifold
                        ),
                        "Internal error: the shape was no longer valid."
                    );
                }

                true
            } else {
                // FIXME: ask the detector if it wants to be removed or not
                false
            }
        });
    }
}

impl<N: Real> ContactManifoldGenerator<N> for TriMeshShapeManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        d: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        fmap1: Option<&Fn(FeatureId) -> FeatureId>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        fmap2: Option<&Fn(FeatureId) -> FeatureId>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if !self.flip {
            if let Some(trimesh) = a.as_shape::<TriMesh<N>>() {
                self.do_update(
                    d, ma, trimesh, fmap1, mb, b, fmap2, prediction, id_alloc, manifold,
                );
                return true;
            }
        } else {
            if let Some(trimesh) = b.as_shape::<TriMesh<N>>() {
                self.do_update(
                    d, mb, trimesh, fmap2, ma, a, fmap1, prediction, id_alloc, manifold,
                );
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
