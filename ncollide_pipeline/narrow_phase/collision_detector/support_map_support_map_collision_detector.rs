use std::any::Any;
use std::marker::PhantomData;
use na::{Translate, Translation};
use math::{Point, Vector};
use entities::inspection::Shape;
use entities::shape::AnnotatedPoint;
use queries::geometry::algorithms::simplex::Simplex;
use queries::geometry::algorithms::gjk::GJKResult;
use queries::geometry::contacts_internal;
use queries::geometry::Contact;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapSupportMapCollisionDetector<P: Point, M, S> {
    simplex:  S,
    contact:  GJKResult<Contact<P>, P::Vect>,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P, M, S> SupportMapSupportMapCollisionDetector<P, M, S>
    where P: Point,
          S: Simplex<AnnotatedPoint<P>> {
    /// Creates a new persistant collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(simplex: S) -> SupportMapSupportMapCollisionDetector<P, M, S> {
        SupportMapSupportMapCollisionDetector {
            simplex:  simplex,
            contact:  GJKResult::Intersection,
            mat_type: PhantomData
        }
    }
}

impl<P, M, S> CollisionDetector<P, M> for SupportMapSupportMapCollisionDetector<P, M, S>
    where P: Point,
          P::Vect: Translate<P>,
          M: Translation<P::Vect> + Any,
          S: Simplex<AnnotatedPoint<P>> {
    #[inline]
    fn update(&mut self,
              _:          &CollisionDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let (Some(sma), Some(smb)) = (a.desc().as_support_map(), b.desc().as_support_map()) {
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
    fn num_colls(&self) -> usize {
        match self.contact {
            GJKResult::Projection(_) => 1,
            _                        => 0
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        match self.contact {
            GJKResult::Projection(ref c) => out_colls.push(c.clone()),
            _                            => ()
        }
    }
}
