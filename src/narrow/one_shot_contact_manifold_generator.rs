use std::num::{One, from_f32};
use nalgebra::na::{AlgebraicVecExt, Vec, Basis, Cross, RotationWithTranslation, Translation,
                   Rotation, Transform};
use narrow::{CollisionDetector, IncrementalContactManifoldGenerator};
use contact::Contact;

/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    priv sub_detector: IncrementalContactManifoldGenerator<CD, N, LV>
}

impl<CD, N, LV, AV, M> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(prediction: N, cd: CD) -> OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(prediction, cd)
        }
    }
}

impl<CD: CollisionDetector<N, LV, M, G1, G2>,
     G1,
     G2,
     N:  Clone + Num + Ord + FromPrimitive + Algebraic,
     LV: Clone + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + ToStr,
     AV: Vec<N> + ToStr,
     M:  Rotation<AV> + Transform<LV> + Translation<LV> + One>
CollisionDetector<N, LV, M, G1, G2> for OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    fn update(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) {
        if self.sub_detector.num_colls() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(m1, g1, m2, g2) {
                Some(coll) => {
                    do coll.normal.orthonormal_subspace_basis |b| {
                        let mut rot_axis: AV = coll.normal.cross(&b);

                        // first perturbation
                        rot_axis = rot_axis * from_f32(0.01).unwrap();

                        let rot_mat: M = m1.rotated_wrt_center(&rot_axis);

                        self.sub_detector.add_new_contacts(&rot_mat, g1, m2, g2);

                        // second perturbation (opposite direction)
                        let rot_mat = m1.rotated_wrt_center(&-rot_axis);

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
    fn num_colls(&self) -> uint {
        self.sub_detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, LV>]) {
        self.sub_detector.colls(out_colls)
    }

    #[inline]
    fn toi(_:    Option<OneShotContactManifoldGenerator<CD, N, LV, AV, M>>,
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &G1,
           m2:   &M,
           g2:   &G2) -> Option<N> {
        CollisionDetector::toi(None::<CD>, m1, dir, dist, g1, m2, g2)
    }
}
