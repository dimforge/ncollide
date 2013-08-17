use std::num::One;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::cross::Cross;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::rotation;
use nalgebra::traits::rotation::Rotation;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::vector_space::VectorSpace;
use narrow::collision_detector::CollisionDetector;
use narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
use contact::Contact;

/// This is an hybrid contact manifold genarator. Whenever a new contact is detected (i.e. when the
/// current manifold is empty) a full manifold is generated. Then, the manifold is incrementally
/// updated by the `IncrementalContactManifoldGenerator`.
pub struct OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    priv sub_detector: IncrementalContactManifoldGenerator<CD, N, LV>
}

impl<CD, N, LV, AV, M> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(cd: CD) -> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(cd)
        }
    }
}

impl<CD: CollisionDetector<N, LV, M, G1, G2>,
     G1,
     G2,
     N:  Clone + DivisionRing + Ord + NumCast,
     LV: Clone + VectorSpace<N> + Cross<AV> + Dot<N> + Norm<N> + ApproxEq<N> + Dim + Basis +
         ToStr,
     AV: ScalarMul<N> + Neg<AV> + ToStr,
     M:  Rotation<AV> + Transform<LV> + Translation<LV> + Translatable<LV, M> + One>
CollisionDetector<N, LV, M, G1, G2> for OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    fn update(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) {
        if self.sub_detector.num_coll() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(m1, g1, m2, g2) {
                Some(coll) => {
                    do coll.normal.orthonormal_subspace_basis |b| {
                        let mut rot_axis: AV = coll.normal.cross(&b);

                        // first perturbation
                        rot_axis.scalar_mul_inplace(&NumCast::from(0.01));

                        let rot_mat: M = rotation::rotated_wrt_center(m1, &rot_axis);

                        self.sub_detector.add_new_contacts(&rot_mat, g1, m2, g2);

                        // second perturbation (opposite direction)
                        let rot_mat = rotation::rotated_wrt_center(m1, &-rot_axis);

                        self.sub_detector.add_new_contacts(&rot_mat, g1, m2, g2);

                        true
                    }

                    self.sub_detector.update_contacts(m1, m2);
                },
                None => { } // no collision
            }
        }
        else {
            // otherwise, let the incremental manifold do its job
            self.sub_detector.update(m1, g1, m2, g2)
        }
    }

    #[inline]
    fn num_coll(&self) -> uint {
        self.sub_detector.num_coll()
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, LV>]) {
        self.sub_detector.colls(out_colls)
    }
}
