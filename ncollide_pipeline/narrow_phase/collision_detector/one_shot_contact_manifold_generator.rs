use std::ops::Mul;
use na::{Cross, Transform, Translation, Rotation};
use na;
use math::{Point, Vector};
use entities::inspection::Repr;
use queries::geometry::Contact;
use narrow_phase::{CollisionDetector, CollisionDispatcher, IncrementalContactManifoldGenerator};


/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[derive(RustcEncodable, RustcDecodable, Clone)]
pub struct OneShotContactManifoldGenerator<P: Point, M, CD> {
    sub_detector: IncrementalContactManifoldGenerator<P, M, CD>
}

impl<P, M, CD> OneShotContactManifoldGenerator<P, M, CD>
    where P:  Point,
          CD: CollisionDetector<P, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(cd: CD) -> OneShotContactManifoldGenerator<P, M, CD> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(cd)
        }
    }
}

impl<P, M, CD> CollisionDetector<P, M> for OneShotContactManifoldGenerator<P, M, CD>
    where P:       Point,
          P::Vect: Cross,
          <P::Vect as Cross>::CrossProductType: Vector<Scalar = <P::Vect as Vector>::Scalar> +
                                                Mul<<P::Vect as Vector>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M:  Transform<P> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType>,
          CD: CollisionDetector<P, M> {
    fn update(&mut self,
              d:  &CollisionDispatcher<P, M>,
              m1: &M,
              g1: &Repr<P, M>,
              m2: &M,
              g2: &Repr<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if self.sub_detector.num_colls() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(d, m1, g1, m2, g2, prediction) {
                Some(Some(coll)) => {
                    na::orthonormal_subspace_basis(&coll.normal, |b| {
                        let mut rot_axis = na::cross(&coll.normal, &b);

                        // first perturbation
                        rot_axis = rot_axis * na::cast::<f64, <P::Vect as Vector>::Scalar>(0.01f64);

                        let rot_mat: M = na::append_rotation_wrt_point(m1, &rot_axis, coll.world1.as_vector());

                        self.sub_detector.add_new_contacts(d, &rot_mat, g1, m2, g2, prediction);

                        // second perturbation (opposite direction)
                        let rot_mat = na::append_rotation_wrt_point(m1, &-rot_axis, coll.world1.as_vector());

                        self.sub_detector.add_new_contacts(d, &rot_mat, g1, m2, g2, prediction);

                        true
                    });

                    self.sub_detector.update_contacts(m1, m2, prediction);

                    true
                },
                Some(None) => true,  // no collision
                None       => false // invalid
            }
        }
        else {
            // otherwise, let the incremental manifold do its job
            self.sub_detector.update(d, m1, g1, m2, g2, prediction)
        }
    }

    #[inline]
    fn num_colls(&self) -> usize {
        self.sub_detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        self.sub_detector.colls(out_colls)
    }
}
