use std::num::{Zero, One};
use nalgebra::traits::translation::Translation;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::AlgebraicVecExt;
use geom::implicit::Implicit;
use geom::reflection::Reflection;
use geom::minkowski_sum;
use geom::minkowski_sum::{AnnotatedPoint, NonTransformableMinkowskiSum};
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::gjk;
use narrow::algorithm::minkowski_sampling;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;
use ray::ray::{Ray, RayCast};

/// Persistant collision detector between two shapes having a support mapping function (i.e. which
/// implement the `Implicit` trait. It is based on the GJK algorithm.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitImplicit<S, G1, G2, N, V> {
    priv simplex:    S,
    priv margin:     N,
    priv prediction: N,
    priv contact:    Option<Contact<N, V>>
}

impl<S, G1, G2, N, V> ImplicitImplicit<S, G1, G2, N, V> {
    /// Creates a new persistant collision detector between two geometries with support mapping
    /// functions. It is initialized with a pre-created simplex.
    pub fn new(margin: N, prediction: N, simplex: S) -> ImplicitImplicit<S, G1, G2, N, V> {
        ImplicitImplicit {
            simplex:    simplex,
            margin:     margin,
            prediction: prediction,
            contact:    None
        }
    }

}

impl<S:  Simplex<N, AnnotatedPoint<V>>,
     G1: Implicit<V, M>,
     G2: Implicit<V, M>,
     N:  Sub<N, N> + Ord + Mul<N, N> + Float + Clone + ToStr,
     V:  AlgebraicVecExt<N> + Clone + ToStr,
     M:  Translation<V> + Transform<V> + Rotate<V> + One>
     CollisionDetector<N, V, M, G1, G2> for ImplicitImplicit<S, G1, G2, N, V> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G1, mb: &M, b: &G2) {
        self.contact = collide(
            ma,
            a,
            mb,
            b,
            &self.margin,
            &self.prediction,
            &mut self.simplex)
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }

    #[inline]
    fn toi(ma: &M, dir: &V, a: &G1, mb: &M, b: &G2) -> Option<N> {
        toi(ma, dir, a, mb, b)
    }
}

/// Computes a contact point between two implicit geometries. For optimizations purposes the
/// objects are artificially enlarged by a small margin. This uses the GJK algorithm to handle
/// penetrations within the margin and a Minkowski sum sampling based algorithm to handle deep
/// penetrations.
///
/// # Arguments:
///   * `g1` - the first implicit shape involved on the collision check
///   * `g2` - the second implicit shape involved on the collision check
///   * `margin` - margin used to enlarge both shapes. This must not be negative. Small or too big
///   values can lead to visible gaps in-between objects. In practice, values on the range `[0.04;
///   0.08]` give good results.
///   * `simplex` - the simplex the GJK algorithm must use. It is reinitialized before being passed
///   to GJK.
pub fn collide<S:  Simplex<N, AnnotatedPoint<V>>,
               G1: Implicit<V, M>,
               G2: Implicit<V, M>,
               N:  Sub<N, N> + Ord + Mul<N, N> + Float + Clone + ToStr,
               V:  AlgebraicVecExt<N> + Clone,
               M:  Translation<V> + One>(
               m1:         &M,
               g1:         &G1,
               m2:         &M,
               g2:         &G2,
               margin:     &N,
               prediction: &N,
               simplex: &mut S)
               -> Option<Contact<N, V>> {
    let mut dir = m1.translation() - m2.translation(); // FIXME: or m2.translation - m1.translation ?

    if dir.is_zero() {
        dir = Zero::zero();
        dir.set(0, One::one());
    }

    simplex.reset(minkowski_sum::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = p1p2.sqnorm();

            if sqn >= (*margin * NumCast::from(2.0) + *prediction) *
                      (*margin * NumCast::from(2.0) + *prediction) {
                return None
            }
            else if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize();

                return Some(
                    Contact::new(
                        p1 + normal * *margin,
                        p2 + normal * (-margin),
                        normal,
                        *margin + *margin - depth)
                    );
            }
        },
        None => { }
    }

    // The point is inside of the CSO: use the fallback algorithm
    match minkowski_sampling::closest_points(m1, g1, m2, g2, margin, simplex) {
        Some((p1, p2)) => {
            let mut normal = p1 - p2;
            let depth      = normal.normalize();

            if depth.is_zero() {
                // FIXME: this seems to happend on some very rare cases which makes the johnson
                // simplex fail.
                // This might be an implementation bug…
                // … as a workaround we just act as if nothing happended…
                None
            }
            else {
                Some(Contact::new(p1, p2, normal, depth))
            }
        }
        None => {
            None // fail!("Both GJK and fallback algorithm failed.")
        }
    }
}

/// Computes the Time Of Impact of two geometries.
///
/// Arguments:
///     * `m1`  - the first geometry transform.
///     * `dir` - the direction of the first geometry movement.
///     * `g1`  - the first geometry.
///     * `m2`  - the second geometry transform.
///     * `g2`  - the second geometry.
pub fn toi<N:  Ord + Num + Float + NumCast + Clone + ToStr,
           V:  AlgebraicVecExt<N> + Clone + ToStr,
           M:  Translation<V> + Transform<V> + Rotate<V>,
           G1: Implicit<V, M>,
           G2: Implicit<V, M>>(
           m1:  &M,
           dir: &V,
           g1:  &G1,
           m2:  &M,
           g2:  &G2)
           -> Option<N> {
    let rg2 = Reflection::new(g2);
    let cso = NonTransformableMinkowskiSum::new(m1, g1, m2, &rg2);

    // m1 will be ignored
    cso.toi_with_ray(m1, &Ray::new(Zero::zero(), -dir))
}
