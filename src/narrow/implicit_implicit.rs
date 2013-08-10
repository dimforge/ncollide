use std::num::One;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::sample::UniformSphereSample;
use nalgebra::traits::translation::{Translation, Translatable};
use geom::implicit::Implicit;
use geom::minkowski_sum;
use geom::minkowski_sum::AnnotatedPoint;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::gjk;
use narrow::algorithm::minkowski_sampling;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;

/// Persistant collision detector between two shapes having a support mapping function (i.e. which
/// implement the `Implicit` trait. It is based on the GJK algorithm.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitImplicit<S, G1, G2, N, V> {
    priv simplex: S,
    priv margin:  N,
    priv contact: Option<Contact<N, V>>
}

impl<S, G1, G2, N, V> ImplicitImplicit<S, G1, G2, N, V> {
    /// Creates a new persistant collision detector between two geometries with support mapping
    /// functions. It is initialized with a pre-created simplex.
    pub fn new(margin: N, simplex: S) -> ImplicitImplicit<S, G1, G2, N, V> {
        ImplicitImplicit {
            simplex: simplex,
            margin:  margin,
            contact: None
        }
    }

}

impl<S:  Simplex<N, AnnotatedPoint<V>>,
     G1: Implicit<V, M>,
     G2: Implicit<V, M>,
     N:  Sub<N, N> + Ord + Mul<N, N> + Float + Clone,
     V:  Norm<N> + VectorSpace<N> + Dot<N> + Dim + UniformSphereSample + Clone,
     M:  Translation<V> + Translatable<V, M> + One>
     CollisionDetector<N, V, M, G1, G2> for ImplicitImplicit<S, G1, G2, N, V> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G1, mb: &M, b: &G2) {
        self.contact = collide_implicit_implicit(ma, a, mb, b, &self.margin, &mut self.simplex)
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
pub fn collide_implicit_implicit<S:  Simplex<N, AnnotatedPoint<V>>,
                                 G1: Implicit<V, M>,
                                 G2: Implicit<V, M>,
                                 N:  Sub<N, N> + Ord + Mul<N, N> + Float + Clone,
                                 V:  Norm<N> + VectorSpace<N> + Dot<N> + Dim +
                                     UniformSphereSample + Clone,
                                 M:  Translation<V> + Translatable<V, M> + One>(
                                 m1:      &M,
                                 g1:      &G1,
                                 m2:      &M,
                                 g2:      &G2,
                                 margin:  &N,
                                 simplex: &mut S)
                                 -> Option<Contact<N, V>> {
    let dir = m1.translation() - m2.translation(); // FIXME: or m2.translation - m1.translation ?

    simplex.reset(minkowski_sum::cso_support_point(m1, g1, m2, g2, dir));

    match gjk::closest_points(m1, g1, m2, g2, simplex) {
        Some((p1, p2)) => {
            let p1p2 = p2 - p1;
            let sqn  = p1p2.sqnorm();

            if sqn >= *margin * *margin * NumCast::from(4.0f64) {
                return None
            }
            else if !sqn.is_zero() {
                let mut normal = p1p2;
                let depth      = normal.normalize();

                return Some(
                    Contact::new(
                        p1 + normal.scalar_mul(margin),
                        p2 + normal.scalar_mul(&-margin),
                        normal,
                        *margin + *margin - depth
                        )
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

            Some(Contact::new(p1, p2, normal, depth))
        }
        None => {
            None // fail!("Both GJK and fallback algorithm failed.")
        }
    }
}
