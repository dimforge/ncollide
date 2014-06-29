#[cfg(not(dim4))]
use nalgebra::na;

use narrow::{CollisionDetector, IncrementalContactManifoldGenerator, Contact};
use math::{Scalar, Vect, Matrix};

#[cfg(not(dim4))]
use math::Orientation;

/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable, Clone)]
pub struct OneShotContactManifoldGenerator<CD> {
    sub_detector: IncrementalContactManifoldGenerator<CD>
}

impl<CD> OneShotContactManifoldGenerator<CD> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(prediction: Scalar, cd: CD) -> OneShotContactManifoldGenerator<CD> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(prediction, cd)
        }
    }
}

impl<CD: CollisionDetector<G1, G2>, G1, G2>
CollisionDetector<G1, G2> for OneShotContactManifoldGenerator<CD> {
    #[cfg(not(dim4))]
    fn update(&mut self, m1: &Matrix, g1: &G1, m2: &Matrix, g2: &G2) {
        if self.sub_detector.num_colls() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(m1, g1, m2, g2) {
                Some(coll) => {
                    na::orthonormal_subspace_basis(&coll.normal, |b| {
                        let mut rot_axis: Orientation = na::cross(&coll.normal, &b);

                        // first perturbation
                        rot_axis = rot_axis * na::cast::<f32, Scalar>(0.01);

                        let rot_mat: Matrix = na::append_rotation_wrt_point(m1, &rot_axis, &coll.world1);

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

    #[cfg(dim4)]
    fn update(&mut self, _: &Matrix, _: &G1, _: &Matrix, _: &G2) {
        fail!("Not yet implemented.")
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.sub_detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        self.sub_detector.colls(out_colls)
    }

    #[inline]
    fn toi(_:    Option<OneShotContactManifoldGenerator<CD>>,
           m1:   &Matrix,
           dir:  &Vect,
           dist: &Scalar,
           g1:   &G1,
           m2:   &Matrix,
           g2:   &G2) -> Option<Scalar> {
        CollisionDetector::toi(None::<CD>, m1, dir, dist, g1, m2, g2)
    }
}
