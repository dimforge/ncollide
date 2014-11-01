use std::num::Zero;
use na::{Transform, Rotate, Translate, Translation, Norm};
use na;
use shape::{AnnotatedPoint, MinkowskiSum, Reflection};
use support_map::{SupportMap, PreferedSamplingDirections};
use support_map;
use geometry::algorithms::simplex::Simplex;
use geometry::algorithms::gjk;
use geometry::algorithms::gjk::{GJKResult, NoIntersection, Intersection, Projection};
use geometry::algorithms::minkowski_sampling;
use narrow_phase::{CollisionDetector, Contact};
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

        self.contact = collide(
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

/// Computes a contact point between two implicit geometries.
///
/// For optimizations purposes the objects are artificially enlarged by a small margin. This uses
/// the GJK algorithm to handle penetrations within the margin and a Minkowski sum sampling based
/// algorithm to handle deep penetrations.
///
/// # Arguments:
///   * `g1` - the first support mapped shape involved on the collision check
///   * `g2` - the second support mapped shape involved on the collision check
///   * `simplex` - the simplex the GJK algorithm must use. It is reinitialized before being passed
///   to GJK.
pub fn collide<N, P, V, M, S, G1, G2>(m1:         &M,
                                      g1:         &G1,
                                      m2:         &M,
                                      g2:         &G2,
                                      prediction: N,
                                      simplex:    &mut S,
                                      init_dir:   Option<V>)
                                      -> GJKResult<Contact<N, P, V>, V>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Translation<V>,
          S:  Simplex<N, AnnotatedPoint<P>>,
          G1: SupportMap<P, V, M> + PreferedSamplingDirections<V, M>,
          G2: SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {
    let mut dir =
        match init_dir {
            None      => m1.translation() - m2.translation(), // FIXME: or m2.translation - m1.translation ?
            Some(dir) => dir
        };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(support_map::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points_with_max_dist(m1, g1, m2, g2, prediction, simplex) {
        Projection((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = na::sqnorm(&p1p2);

            if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize();

                return Projection(Contact::new(p1, p2, normal, -depth));
            }
        },
        NoIntersection(dir) => return NoIntersection(dir),
        Intersection        => { } // fallback
    }

    // The point is inside of the CSO: use the fallback algorithm
    match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2, normal)) => {
            let depth = na::dot(&(p1 - p2), &normal);

            Projection(Contact::new(p1, p2, normal, depth))
        }
        None => {
            NoIntersection(na::zero()) // panic!("Both GJK and fallback algorithm failed.")
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
