use na::{Cross, Transform, Translation, Rotation};
use na;
use math::{Scalar, Point, Vect};
use entities::inspection::Repr;
use queries::geometry::Contact;
use narrow_phase::{CollisionDetector, CollisionDispatcher, IncrementalContactManifoldGenerator};


/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[derive(RustcEncodable, RustcDecodable, Clone)]
pub struct OneShotContactManifoldGenerator<N, P, V, CD> {
    sub_detector: IncrementalContactManifoldGenerator<N, P, V, CD>
}

#[old_impl_check]
impl<N, P, V, M, CD> OneShotContactManifoldGenerator<N, P, V, CD>
    where CD: CollisionDetector<N, P, V, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(prediction: N, cd: CD) -> OneShotContactManifoldGenerator<N, P, V, CD> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(prediction, cd)
        }
    }
}

impl<N, P, V, AV, M, CD> CollisionDetector<N, P, V, M> for OneShotContactManifoldGenerator<N, P, V, CD>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Cross<Output = AV>,
          AV: Vect<N>,
          M:  Transform<P> + Translation<V> + Rotation<AV>,
          CD: CollisionDetector<N, P, V, M> {
    fn update(&mut self,
              d:  &CollisionDispatcher<N, P, V, M>,
              m1: &M,
              g1: &Repr<N, P, V, M>,
              m2: &M,
              g2: &Repr<N, P, V, M>)
              -> bool {
        if self.sub_detector.num_colls() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(d, m1, g1, m2, g2) {
                Some(Some(coll)) => {
                    na::orthonormal_subspace_basis(&coll.normal, |b| {
                        let mut rot_axis = na::cross(&coll.normal, &b);

                        // first perturbation
                        rot_axis = rot_axis * na::cast::<f64, N>(0.01);

                        let rot_mat: M = na::append_rotation_wrt_point(m1, &rot_axis, coll.world1.as_vec());

                        self.sub_detector.add_new_contacts(d, &rot_mat, g1, m2, g2);

                        // second perturbation (opposite direction)
                        let rot_mat = na::append_rotation_wrt_point(m1, &-rot_axis, coll.world1.as_vec());

                        self.sub_detector.add_new_contacts(d, &rot_mat, g1, m2, g2);

                        true
                    });

                    self.sub_detector.update_contacts(m1, m2);

                    true
                },
                Some(None) => true,  // no collision
                None       => false // invalid
            }
        }
        else {
            // otherwise, let the incremental manifold do its job
            self.sub_detector.update(d, m1, g1, m2, g2)
        }
    }

    #[inline]
    fn num_colls(&self) -> usize {
        self.sub_detector.num_colls()
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        self.sub_detector.colls(out_colls)
    }
}
