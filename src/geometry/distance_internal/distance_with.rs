//! Trait used to implement the `distance` function.

use na::Translate;
use na;
use shape::{Ball, Plane, Cuboid, Capsule, Cone, Cylinder, Convex, Compound, Mesh, Segment,
            Triangle};
use geometry::distance_internal;
use math::{Scalar, Point, Vect, Isometry};

/// Trait implemented by object that can be tested for distance with another one.
pub trait DistanceTo<N, P, V, M, Sized? G> for Sized? {
    /// Computes the minimum distance separating two shapes.
    ///
    /// Returns `0.0` if the objects are touching or penetrating.
    fn distance(m1: &M, g1: &Self, m2: &M, g2: &G) -> N;
}

/// Computes the minimum distance separating two shapes.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn distance<N, P, V, M, Sized? G1, Sized? G2>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V>,
          G1: DistanceTo<N, P, V, M, G2> {
    DistanceTo::distance(m1, g1, m2, g2)
}

/*
 *
 *
 * Impls follow.
 *
 *
 */
macro_rules! impl_distance_with(
    ($name: ident | $g1: ty, $g2: ty) => {
        impl<N, P, V, M> DistanceTo<N, P, V, M, $g2> for $g1
            where N: Scalar,
                  P: Point<N, V>,
                  V: Vect<N> + Translate<P> ,
                  M: Isometry<N, P, V> {
            #[inline]
            fn distance(m1: &M, g1: &$g1, m2: &M, g2: &$g2) -> N {
                distance_internal::$name(m1, g1, m2, g2)
            }
        }
    }
);

apply_with_mixed_args!(impl_distance_with,
                       plane_against_support_map       |
                       support_map_against_plane       |
                       support_map_against_support_map |
                       concave_shape_against_shape     |
                       shape_against_concave_shape);

impl<N, P, V, M> DistanceTo<N, P, V, M, Ball<N>> for Ball<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Isometry<N, P, V> {
    #[inline]
    fn distance(m1: &M, g1: &Ball<N>, m2: &M, g2: &Ball<N>) -> N {
        let p1 = m1.translate(&na::orig());
        let p2 = m2.translate(&na::orig());
        distance_internal::ball_against_ball(&p1, g1, &p2, g2)
    }
}

/* FIXME: DST: ICE
impl<N, P, V, M> DistanceTo<N, P, V, M, Shape<N, P, V, M> + Send + Sync> for Shape<N, P, V, M> + Send + Sync
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn distance(m1: &M, g1: &Shape<N, P, V, M> + Send + Sync,
                m2: &M, g2: &Shape<N, P, V, M> + Send + Sync)
                -> N {
        unimplemented!()
        // distance_internal::$name(m1, g1, m2, g2)
    }
}
*/
