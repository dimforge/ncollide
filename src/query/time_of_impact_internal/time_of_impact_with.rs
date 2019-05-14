//! Trait used to implement the `time_of_impact` function.

use std::sync::Arc;
use na::Translate;
use na;
use shape::{Ball, Plane, Cuboid, Capsule, Cone, Cylinder, ConvexHull, Compound, Mesh, Segment,
                      Triangle};
use traits::Shape;
use time_of_impact_internal;
use math::{Scalar, Point, Vector, Isometry};

/// Trait implemented by object that can have their Time Of Impact under translational movement
/// computed.
pub trait TimeOfImpactWith<N, P, V, M, G: ?Sized> for Sized? {
    /// Computes the Time Of Impact separating two shapes.
    ///
    /// Returns `None` if they never move.
    fn time_of_impact(m1: &Isometry<N>, vel1: &V, g1: &Self, m2: &Isometry<N>, vel2: &V, g2: &G) -> Option<N>;
}

/// Computes the Time Of Impact separating two shapes under translational movement.
pub fn time_of_impact<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &Isometry<N>, vel1: &V, g1: &G1,
                                                          m2: &Isometry<N>, vel2: &V, g2: &G2)
                                                          -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vector<N> + Translate<P>,
          M:  Isometry<N, P, V>,
          G1: TimeOfImpactWith<N, P, V, M, G2> {
    TimeOfImpactWith::time_of_impact(m1, vel1, g1, m2, vel2, g2)
}

/*
 *
 *
 * Impls follow.
 *
 *
 */
macro_rules! impl_time_of_impact_with(
    ($name: ident | $g1: ty, $g2: ty) => {
        impl<N, P, V, M> TimeOfImpactWith<N, P, V, M, $g2> for $g1
            where N: Scalar,
                  N: RealField<N, V>,
                  N: RealField<N> + Translate<P> ,
                  M: Isometry<N, P, V> {
            #[inline]
            fn time_of_impact(m1: &Isometry<N>, vel1: &V, g1: &$g1, m2: &Isometry<N>, vel2: &V, g2: &$g2) -> Option<N> {
                time_of_impact_internal::$name(m1, vel1, g1, m2, vel2, g2)
            }
        }
    }
);

apply_with_mixed_args!(impl_time_of_impact_with,
                       plane_against_support_map       |
                       support_map_against_plane       |
                       support_map_against_support_map |
                       concave_shape_against_shape     |
                       shape_against_concave_shape);

impl<N, P, V, M> TimeOfImpactWith<N, P, V, M, Ball<N>> for Ball<N>
    where N: Scalar,
          N: RealField<N, V>,
          N: RealField<N>,
          M: Isometry<N, P, V> {
    #[inline]
    fn time_of_impact(m1: &Isometry<N>, vel1: &V, g1: &Ball<N>, m2: &Isometry<N>, vel2: &V, g2: &Ball<N>) -> Option<N> {
        let p1 = m1.translate(&Point::origin());
        let p2 = m2.translate(&Point::origin());
        time_of_impact_internal::ball_against_ball(&p1, vel1, g1, &p2, vel2, g2)
    }
}

/* FIXME: DST: ICE
impl<N, P, V, M> TimeOfImpactWith<N, P, V, M, Shape<N, P, V, M> + Send + Sync> for Shape<N, P, V, M> + Send + Sync
    where N: Scalar,
          N: RealField<N, V>,
          N: RealField<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn time_of_impact(m1: &Isometry<N>, g1: &Shape<N, P, V, M> + Send + Sync,
                m2: &Isometry<N>, g2: &Shape<N, P, V, M> + Send + Sync)
                -> N {
        unimplemented!()
        // time_of_impact_internal::$name(m1, g1, m2, g2)
    }
}
*/
