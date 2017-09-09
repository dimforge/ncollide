use alga::linear::{FiniteDimInnerSpace, Rotation};
use na;
use math::{Point, Isometry};
use geometry::shape::Shape;
use geometry::query::Contact;
use narrow_phase::{ContactGenerator, ContactDispatcher, IncrementalContactManifoldGenerator};


/// Contact manifold generator producing a full manifold at the first update.
///
/// Whenever a new contact is detected (i.e. when the current manifold is empty) a full manifold is
/// generated. Then, the manifold is incrementally updated by an
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct OneShotContactManifoldGenerator<P: Point, M, CD> {
    sub_detector: IncrementalContactManifoldGenerator<P, M, CD>
}

impl<P, M, CD> OneShotContactManifoldGenerator<P, M, CD>
    where P:  Point,
          CD: ContactGenerator<P, M> {
    /// Creates a new one shot contact manifold generator.
    pub fn new(cd: CD) -> OneShotContactManifoldGenerator<P, M, CD> {
        OneShotContactManifoldGenerator {
            sub_detector: IncrementalContactManifoldGenerator::new(cd)
        }
    }
}

static ROT_ERROR: &'static str =
    "The provided transformation does not have enough \
     rotational degree of freedom for to work ith the \
     one-shot contact manifold generator.";
static TR_ERROR: &'static str =
    "The provided transformation does not have enough    \
     translational degree of freedom for to work ith the \
     one-shot contact manifold generator.";

impl<P, M, CD> ContactGenerator<P, M> for OneShotContactManifoldGenerator<P, M, CD>
    where P:  Point,
          M:  Isometry<P>,
          CD: ContactGenerator<P, M> {
    fn update(&mut self,
              d:  &ContactDispatcher<P, M>,
              m1: &M,
              g1: &Shape<P, M>,
              m2: &M,
              g2: &Shape<P, M>,
              prediction: P::Real)
              -> bool {
        if self.sub_detector.num_contacts() == 0 {
            // do the one-shot manifold generation
            match self.sub_detector.get_sub_collision(d, m1, g1, m2, g2, prediction) {
                Some(Some(coll)) => {
                    P::Vector::orthonormal_subspace_basis(&[coll.normal], |b| {
                        let perturbation = M::Rotation::scaled_rotation_between(&coll.normal, &b, na::convert(0.01))
                                           .expect(ROT_ERROR);
                        let shifted_m1   = m1.append_rotation_wrt_point(&perturbation, &coll.world1)
                                           .expect(TR_ERROR);

                        let _ = self.sub_detector.add_new_contacts(d, &shifted_m1, g1, m2, g2, prediction);

                        // second perturbation (opposite direction)
                        let shifted_m1 = m1.append_rotation_wrt_point(&na::inverse(&perturbation), &coll.world1)
                                         .expect(TR_ERROR);
                        let _ = self.sub_detector.add_new_contacts(d, &shifted_m1, g1, m2, g2, prediction);

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
    fn num_contacts(&self) -> usize {
        self.sub_detector.num_contacts()
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        self.sub_detector.contacts(out_contacts)
    }
}
