use std::num::{Zero, One};
use nalgebra::na::{Cast, Translation, Rotate, Transform, AlgebraicVecExt};
use nalgebra::na;
use geom::{Reflection, AnnotatedPoint, MinkowskiSum};
use implicit::Implicit;
use implicit;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::gjk;
use narrow::algorithm::gjk::{GJKResult, NoIntersection, Intersection, Projection};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use narrow::algorithm::minkowski_sampling;
use narrow::CollisionDetector;
use contact::Contact;
use ray::{Ray, RayCast};

/// Persistent collision detector between two shapes having a support mapping function.
///
/// It is based on the GJK algorithm.  This detector generates only one contact point. For a full
/// manifold generation, see `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct ImplicitImplicit<N, V, S, G1, G2> {
    priv simplex:       S,
    priv prediction:    N,
    priv contact:       GJKResult<Contact<N, V>, V>
}

impl<N: Clone, V: Clone, S: Clone, G1, G2> Clone for ImplicitImplicit<N, V, S, G1, G2> {
    fn clone(&self) -> ImplicitImplicit<N, V, S, G1, G2> {
        ImplicitImplicit {
            simplex:    self.simplex.clone(),
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<N, V, S, G1, G2> ImplicitImplicit<N, V, S, G1, G2> {
    /// Creates a new persistent collision detector between two geometries with support mapping
    /// functions.
    ///
    /// It is initialized with a pre-created simplex.
    pub fn new(prediction: N, simplex: S) -> ImplicitImplicit<N, V, S, G1, G2> {
        ImplicitImplicit {
            simplex:    simplex,
            prediction: prediction,
            contact:    Intersection
        }
    }

}

impl<S:  Simplex<N, AnnotatedPoint<V>>,
     G1: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
     G2: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
     N:  Sub<N, N> + Ord + Mul<N, N> + Float + Cast<f32> + Clone,
     V:  AlgebraicVecExt<N> + Clone,
     M:  Translation<V> + Transform<V> + Rotate<V> + One>
     CollisionDetector<N, V, M, G1, G2> for ImplicitImplicit<N, V, S, G1, G2> {
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
            &self.prediction,
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
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Projection(ref c) => out_colls.push(c.clone()),
            _                 => ()
        }
    }

    #[inline]
    fn toi(_:   Option<ImplicitImplicit<N, V, S, G1, G2>>,
           ma:  &M,
           dir: &V,
           _:   &N,
           a:   &G1,
           mb:  &M,
           b:   &G2) -> Option<N> {
        toi(ma, dir, a, mb, b)
    }
}

/// Computes a contact point between two implicit geometries.
///
/// For optimizations purposes the objects are artificially enlarged by a small margin. This uses
/// the GJK algorithm to handle penetrations within the margin and a Minkowski sum sampling based
/// algorithm to handle deep penetrations.
///
/// # Arguments:
///   * `g1` - the first implicit shape involved on the collision check
///   * `g2` - the second implicit shape involved on the collision check
///   * `simplex` - the simplex the GJK algorithm must use. It is reinitialized before being passed
///   to GJK.
pub fn collide<S:  Simplex<N, AnnotatedPoint<V>>,
               G1: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
               G2: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
               N:  Sub<N, N> + Ord + Mul<N, N> + Float + Cast<f32> + Clone,
               V:  AlgebraicVecExt<N> + Clone,
               M:  Translation<V> + One>(
               m1:         &M,
               g1:         &G1,
               m2:         &M,
               g2:         &G2,
               prediction: &N,
               simplex:    &mut S,
               init_dir:   Option<V>)
               -> GJKResult<Contact<N, V>, V> {
    let mut dir = 
        match init_dir {
            None      => m1.translation() - m2.translation(), // FIXME: or m2.translation - m1.translation ?
            Some(dir) => dir
        };

    if dir.is_zero() {
        dir.set(0, na::one());
    }

    simplex.reset(implicit::cso_support_point_without_margin(m1, g1, m2, g2, dir));

    let margin1  = g1.margin();
    let margin2  = g2.margin();
    let max_dist = margin1 + margin2 + *prediction;

    match gjk::closest_points_without_margin_with_max_dist(m1, g1, m2, g2, &max_dist, simplex) {
        Projection((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = na::sqnorm(&p1p2);

            if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize();

                return Projection(
                    Contact::new(
                        p1 + normal * margin1,
                        p2 + normal * (-margin2),
                        normal,
                        margin1 + margin2 - depth)
                    );
            }
        },
        NoIntersection(dir) => return NoIntersection(dir),
        Intersection        => { } // fallback
    }

    // The point is inside of the CSO: use the fallback algorithm
    match minkowski_sampling::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2)) => {
            let mut normal = p1 - p2;
            let depth      = normal.normalize();

            if depth.is_zero() {
                // FIXME: this seems to happend on some very rare cases which makes the johnson
                // simplex fail.
                // This might be an implementation bug…
                // … as a workaround we just act as if nothing happended…
                NoIntersection(na::zero())
            }
            else {
                Projection(Contact::new(p1, p2, normal, depth))
            }
        }
        None => {
            NoIntersection(na::zero()) // fail!("Both GJK and fallback algorithm failed.")
        }
    }
}

/// Computes the Time Of Impact of two geometries.
///
/// # Arguments:
/// * `m1`  - the first geometry transform.
/// * `dir` - the direction of the first geometry movement.
/// * `g1`  - the first geometry.
/// * `m2`  - the second geometry transform.
/// * `g2`  - the second geometry.
pub fn toi<N:  Ord + Num + Float + Cast<f32> + Clone,
           V:  AlgebraicVecExt<N> + Clone,
           M:  Translation<V> + Transform<V> + Rotate<V>,
           G1: Implicit<N, V, M>,
           G2: Implicit<N, V, M>>(
           m1:  &M,
           dir: &V,
           g1:  &G1,
           m2:  &M,
           g2:  &G2)
           -> Option<N> {
    let rg2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &rg2);

    cso.toi_with_ray(&Ray::new(Zero::zero(), -dir))
}
