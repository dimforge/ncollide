use na::{Translate, Translation};
use shape::AnnotatedPoint;
use support_map::{SupportMap, PreferedSamplingDirections};
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::gjk::GJKResult;
use geometry::contacts_internal;
use narrow_phase::CollisionDetector;
use geometry::Contact;
use math::{Scalar, Point, Vect};


/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[derive(RustcEncodable, RustcDecodable)]
pub struct SupportMapSupportMap<N, P, V, S, G1, G2> {
    simplex:       S,
    prediction:    N,
    contact:       GJKResult<Contact<N, P, V>, V>
}

impl<N: Clone, P: Clone, V: Clone, S: Clone, G1, G2> Clone for SupportMapSupportMap<N, P, V, S, G1, G2> {
    fn clone(&self) -> SupportMapSupportMap<N, P, V, S, G1, G2> {
        SupportMapSupportMap {
            simplex:    self.simplex.clone(),
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<N, P, V, S, G1, G2> SupportMapSupportMap<N, P, V, S, G1, G2> {
    /// Creates a new persistent collision detector between two shapes with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(prediction: N, simplex: S) -> SupportMapSupportMap<N, P, V, S, G1, G2> {
        SupportMapSupportMap {
            simplex:    simplex,
            prediction: prediction,
            contact:    GJKResult::Intersection
        }
    }

}

impl<N, P, V, S, M, G1, G2> CollisionDetector<N, P, V, M, G1, G2> for SupportMapSupportMap<N, P, V, S, G1, G2>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Translation<V>,
          S:  Simplex<N, AnnotatedPoint<P>>,
          G1: SupportMap<P, V, M> + PreferedSamplingDirections<V, M>,
          G2: SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G1, mb: &M, b: &G2) {
        let initial_direction = match self.contact {
            GJKResult::NoIntersection(ref separator) => Some(separator.clone()),
            GJKResult::Projection(ref contact)       => Some(contact.normal.clone()),
            GJKResult::Intersection                  => None
        };

        self.contact = contacts_internal::support_map_against_support_map_with_params(
            ma,
            a,
            mb,
            b,
            self.prediction,
            &mut self.simplex,
            initial_direction)
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
