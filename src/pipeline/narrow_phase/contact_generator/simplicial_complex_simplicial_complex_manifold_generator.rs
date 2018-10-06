use bounding_volume::{AABB, BoundingVolume};
use math::{Isometry, Vector};
use na::Real;
use pipeline::narrow_phase::{ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator};
use query::{ContactKinematic, ContactManifold, ContactPrediction, visitors::AABBSetsInterferencesCollector};
use shape::{FeatureId, Shape, SimplicialComplex};
use std::collections::{hash_map::Entry, HashMap};
use utils::DeterministicState;
use utils::IdAllocator;


/// Collision detector between a concave shape and another shape.
pub struct SimplicialComplexSimplicialComplexManifoldGenerator<N: Real> {
    interferences: Vec<(FeatureId, FeatureId)>,
    manifold: ContactManifold<N>,
}

impl<N: Real> SimplicialComplexSimplicialComplexManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> SimplicialComplexSimplicialComplexManifoldGenerator<N> {
        SimplicialComplexSimplicialComplexManifoldGenerator {
            interferences: Vec::new(),
            // FIXME: use a manifold that uses feature-based matching.
            manifold: ContactManifold::new(),
        }
    }
}

impl<N: Real> SimplicialComplexSimplicialComplexManifoldGenerator<N> {
    fn do_update(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        id1: usize,
        m1: &Isometry<N>,
        g1: &SimplicialComplex<N>,
        id2: usize,
        m2: &Isometry<N>,
        g2: &SimplicialComplex<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) {
        self.manifold.save_cache_and_clear(id_alloc);

        // Find new collisions
        let ls_m2 = m1.inverse() * m2;
        // For transforming AABBs from g2 in the local space of g1.
        let ls_m2_abs_rot = ls_m2.rotation.to_rotation_matrix().matrix().abs();

        /*
        {
            let mut visitor =
                AABBSetsInterferencesCollector::new(&ls_m2, &ls_m2_abs_rot, &mut self.interferences);
            g1.bvh().visit_bvtt(g2.bvh(), &mut visitor);
        }
        */

        for id in self.interferences.drain(..) {
            // The simplest method here is to let ContactKinematic compute the contact here.
            let mut kinematic = ContactKinematic::new();
            if !g1.local_approximation(id.0, kinematic.approx1_mut()) {
                continue;
            }

            if !g2.local_approximation(id.1, kinematic.approx2_mut()) {
                continue;
            }

            if let Some(contact) = kinematic.contact(m1, m2, &Vector::y_axis()) {
                // FIXME: check contacts on triangles actually lie on the triangle relative interior.
                let _ = self.manifold.push(contact, kinematic, id_alloc);
            }
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N>
for SimplicialComplexSimplicialComplexManifoldGenerator<N> {
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
        unimplemented!()
        /*
        if let (Some(sa), Some(sb)) = (a.as_shape::<SimplicialComplex<N>>(), b.as_shape::<SimplicialComplex<N>>()) {
            self.do_update(d, ida, ma, sa, idb, mb, sb, prediction, id_alloc);
            true
        } else {
            false
        }
        */
    }

    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        out.push(&self.manifold)
    }
}
