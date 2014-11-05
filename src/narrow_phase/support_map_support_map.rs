use na::{Transform, Rotate, Translate, Translation};
use na;
use shape::{AnnotatedPoint, MinkowskiSum, Reflection};
use support_map::{SupportMap, PreferedSamplingDirections};
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::gjk::{GJKResult, NoIntersection, Intersection, Projection};
use geometry::contacts_internal;
use narrow_phase::CollisionDetector;
use geometry::Contact;
use ray::{Ray, LocalRayCast};
use math::{Scalar, Point, Vect};


/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
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
    /// Creates a new persistent collision detector between two geometries with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(prediction: N, simplex: S) -> SupportMapSupportMap<N, P, V, S, G1, G2> {
        SupportMapSupportMap {
            simplex:    simplex,
            prediction: prediction,
            contact:    Intersection
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
            NoIntersection(ref separator) => Some(separator.clone()),
            Projection(ref contact)       => Some(contact.normal.clone()),
            Intersection                  => None
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
            Projection(_) => 1,
            _             => 0
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            Projection(ref c) => out_colls.push(c.clone()),
            _                 => ()
        }
    }
}

/// Computes the Time Of Impact of two geometries.
///
/// # Arguments:
/// * `m1`  - the first shape transform.
/// * `dir` - the direction of the first shape movement.
/// * `g1`  - the first shape.
/// * `m2`  - the second shape transform.
/// * `g2`  - the second shape.
pub fn toi<N, P, V, M, G1, G2>(m1: &M, dir: &V, g1: &G1, m2: &M, g2: &G2) -> Option<N>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Rotate<V> + Transform<P>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_with_ray(&Ray::new(na::orig(), -*dir), true)
}

/// Computes the Time Of Impact of two geometries.
///
/// # Arguments:
/// * `m1`  - the first shape transform.
/// * `dir` - the direction of the first shape movement.
/// * `g1`  - the first shape.
/// * `m2`  - the second shape transform.
/// * `g2`  - the second shape.
pub fn toi_and_normal<N, P, V, M, G1, G2>( m1: &M, dir: &V, g1: &G1, m2: &M, g2: &G2) -> Option<(N, V)>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N>,
          M:  Rotate<V> + Transform<P>,
          G1: SupportMap<P, V, M>,
          G2: SupportMap<P, V, M> {
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_and_normal_with_ray(&Ray::new(na::orig(), -*dir), true).map(|i| (i.toi, -i.normal))
}
