use std::marker::PhantomData;
use math::{Point, Isometry};
use geometry::shape::{Shape, AnnotatedPoint};
use geometry::query::algorithms::simplex::Simplex;
use geometry::query::algorithms::gjk::GJKResult;
use geometry::query::contacts_internal;
use geometry::query::Contact;
use narrow_phase::{ContactGenerator, ContactDispatcher};


/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapSupportMapContactGenerator<P: Point, M, S> {
    simplex:  S,
    contact:  GJKResult<Contact<P>, P::Vector>,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P, M, S> SupportMapSupportMapContactGenerator<P, M, S>
    where P: Point,
          S: Simplex<AnnotatedPoint<P>> {
    /// Creates a new persistant collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapContactGenerator<P, M, S> {
        SupportMapSupportMapContactGenerator {
            simplex:  simplex,
            contact:  GJKResult::Intersection,
            mat_type: PhantomData
        }
    }
}

impl<P, M, S: Sync + Send> ContactGenerator<P, M> for SupportMapSupportMapContactGenerator<P, M, S>
    where P: Point,
          M: Isometry<P>,
          S: Simplex<AnnotatedPoint<P>> {
    #[inline]
    fn update(&mut self,
              _:          &ContactDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: P::Real)
              -> bool {
        if let (Some(sma), Some(smb)) = (a.as_support_map(), b.as_support_map()) {
            let initial_direction = match self.contact {
                GJKResult::NoIntersection(ref separator) => Some(separator.clone()),
                GJKResult::Projection(ref contact)       => Some(contact.normal.clone()),
                GJKResult::Intersection                  => None,
                GJKResult::Proximity(_)                  => unreachable!()
            };

            self.contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                prediction,
                &mut self.simplex,
                initial_direction);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        match self.contact {
            GJKResult::Projection(_) => 1,
            _                        => 0
        }
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        match self.contact {
            GJKResult::Projection(ref c) => out_contacts.push(c.clone()),
            _                            => ()
        }
    }
}
