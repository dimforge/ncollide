use bounding_volume::{self, BoundingVolume};
use math::Isometry;
use na::{self, Real};
use partitioning::BVH;
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{visitors::BoundingVolumeInterferencesCollector, ContactManifold, ContactPrediction};
use shape::{CompositeShape, FeatureId, Shape, TriMesh, Triangle};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;

/// Collision detector between a concave shape and another shape.
pub struct TriMeshShapeManifoldGenerator<N: Real> {
    sub_detectors: HashMap<usize, (ContactAlgorithm<N>, ContactManifold<N>), DeterministicState>,
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
        id1: usize,
        m1: &Isometry<N>,
        g1: &TriMesh<N>,
        id2: usize,
        m2: &Isometry<N>,
        g2: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        flip: bool,
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
                    if flip {
                        new_detector = dispatcher.get_contact_algorithm(g2, &tri)
                    } else {
                        new_detector = dispatcher.get_contact_algorithm(&tri, g2)
                    }

                    if let Some(new_detector) = new_detector {
                        let _ = entry.insert((new_detector, ContactManifold::new()));
                    }
                }
            }
        }

        // Update all collisions
        self.sub_detectors.retain(|key, detector| {
            let face = &g1.faces()[*key];

            if ls_aabb2.intersects(&g1.aabb_at(*key)) {
                let tri = Triangle::new(
                    g1.points()[face.indices.x],
                    g1.points()[face.indices.y],
                    g1.points()[face.indices.z],
                );

                detector.1.save_cache_and_clear(id_alloc);

                if flip {
                    assert!(
                        detector.0.update_to(
                            dispatcher,
                            id2,
                            m2,
                            g2,
                            id1,
                            m1,
                            &tri,
                            prediction,
                            id_alloc,
                            &mut detector.1
                        ),
                        "Internal error: the shape was no longer valid."
                    );

                    for contact in detector.1.contacts_mut() {
                        match &mut contact.kinematic.approx2_mut().feature {
                            FeatureId::Vertex(i) => *i = face.indices[*i],
                            FeatureId::Edge(i) => *i = face.edges[*i],
                            FeatureId::Face(i) => {
                                if *i == 0 {
                                    *i = *key
                                } else {
                                    *i = *key + g1.faces().len()
                                }
                            }
                            FeatureId::Unknown => {}
                        }
                    }
                } else {
                    assert!(
                        detector.0.update_to(
                            dispatcher,
                            id1,
                            m1,
                            &tri,
                            id2,
                            m2,
                            g2,
                            prediction,
                            id_alloc,
                            &mut detector.1
                        ),
                        "Internal error: the shape was no longer valid."
                    );

                    for contact in detector.1.contacts_mut() {
                        match &mut contact.kinematic.approx1_mut().feature {
                            FeatureId::Vertex(i) => *i = face.indices[*i],
                            FeatureId::Edge(i) => *i = face.edges[*i],
                            FeatureId::Face(i) => {
                                if *i == 0 {
                                    *i = *key
                                } else {
                                    *i = *key + g1.faces().len()
                                }
                            }
                            FeatureId::Unknown => {}
                        }
                    }
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
        if !self.flip {
            if let Some(trimesh) = a.as_shape::<TriMesh<N>>() {
                self.do_update(d, ida, ma, trimesh, idb, mb, b, prediction, id_alloc, false);
                return true;
            }
        } else {
            if let Some(trimesh) = b.as_shape::<TriMesh<N>>() {
                self.do_update(d, idb, mb, trimesh, ida, ma, a, prediction, id_alloc, true);
                return true;
            }
        }

        return false;
    }

    fn update_to(
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
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        unimplemented!()
    }

    fn num_contacts(&self) -> usize {
        let mut res = 0;

        for detector in &self.sub_detectors {
            res = res + (detector.1).1.len()
        }

        res
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        for detector in &self.sub_detectors {
            out.push(&(detector.1).1);
        }
    }
}
