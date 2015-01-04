use na::{Translate, Translation};
use math::{Scalar, Point, Vect};
use entities::inspection;
use entities::inspection::Repr;
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
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub struct SupportMapSupportMap<N, P, V, M, S> {
    simplex:       S,
    prediction:    N,
    contact:       GJKResult<Contact<N, P, V>, V>
}

impl<N, P, V, M, S> SupportMapSupportMap<N, P, V, M, S>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          S: Simplex<N, AnnotatedPoint<P>> {
    /// Creates a new persistant collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(prediction: N, simplex: S) -> SupportMapSupportMap<N, P, V, M, S> {
        SupportMapSupportMap {
            simplex:    simplex,
            prediction: prediction,
            contact:    GJKResult::Intersection
        }
    }
}

impl<N, P, V, M, S> CollisionDetector<N, P, V, M> for SupportMapSupportMap<N, P, V, M, S>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Translation<V>,
          S: Simplex<N, AnnotatedPoint<P>> {
    #[inline]
    fn update(&mut self,
              _:  &CollisionDispatcher<N, P, V, M>,
              ma: &M,
              a:  &Repr<N, P, V, M>,
              mb: &M,
              b:  &Repr<N, P, V, M>)
              -> bool {
        if let (Some(sma), Some(smb)) = (inspection::maybe_as_support_map(a), inspection::maybe_as_support_map(b)) {
            let initial_direction = match self.contact {
                GJKResult::NoIntersection(ref separator) => Some(separator.clone()),
                GJKResult::Projection(ref contact)       => Some(contact.normal.clone()),
                GJKResult::Intersection                  => None
            };

            self.contact = contacts_internal::support_map_against_support_map_with_params(
                ma,
                sma,
                mb,
                smb,
                self.prediction,
                &mut self.simplex,
                initial_direction);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        match self.contact {
            GJKResult::Projection(_) => 1,
            _ => 0
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            GJKResult::Projection(ref c) => out_colls.push(c.clone()),
            _ => ()
        }
    }
}
