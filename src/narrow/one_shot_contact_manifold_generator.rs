use std::num::One;
use nalgebra::na::{Cast, AlgebraicVecExt, Vec, Cross, Translation,
                   Rotation, Transform};
use nalgebra::na;
use narrow::{CollisionDetector, IncrementalContactManifoldGenerator};
use contact::Contact;

/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable, Clone)]
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
     N:  Clone + Num + Ord + Algebraic + Cast<f32>,
     LV: Clone + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N>,
     AV: Vec<N>,
     M:  Rotation<AV> + Transform<LV> + Translation<LV> + One>
CollisionDetector<N, LV, M, G1, G2> for OneShotContactManifoldGenerator<CD, N, LV, AV, M> {
    fn update(&mut self, m1: &M, g1: &G1, m2: &M, g2: &G2) {
        if self.sub_detector.num_colls() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(m1, g1, m2, g2) {
                Some(coll) => {
                    na::orthonormal_subspace_basis(&coll.normal, |b| {
                        let mut rot_axis: AV = na::cross(&coll.normal, &b);

                        // first perturbation
                        rot_axis = rot_axis * na::cast(0.01);

                        let rot_mat: M = na::append_rotation_wrt_point(m1, &rot_axis, &coll.world1);

                        self.sub_detector.add_new_contacts(&rot_mat, g1, m2, g2);

                        // second perturbation (opposite direction)
                        let rot_mat = na::append_rotation_wrt_point(m1, &-rot_axis, &coll.world1);

                        self.sub_detector.add_new_contacts(&rot_mat, g1, m2, g2);

                        true
                    });

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
