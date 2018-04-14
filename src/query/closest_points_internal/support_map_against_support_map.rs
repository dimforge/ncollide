use num::Zero;

use alga::linear::Translation;
use na;
use shape::{self, AnnotatedPoint, SupportMap};
use query::algorithms::gjk::GJKResult;
use query::algorithms::gjk;
use query::algorithms::{JohnsonSimplex, Simplex, VoronoiSimplex2, VoronoiSimplex3};
use query::ClosestPoints;
use math::{Isometry, Point};

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> ClosestPoints<N>
where
    N: Real,
    M: Isometry<P>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    if na::dimension::<Vector<N>>() == 2 {
        match support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            prediction,
            &mut VoronoiSimplex2::new(),
            None,
        ) {
            GJKResult::Projection(pts, _) => ClosestPoints::WithinMargin(pts.0, pts.1),
            GJKResult::NoIntersection(_) => ClosestPoints::Disjoint,
            GJKResult::Intersection => ClosestPoints::Intersecting,
            GJKResult::Proximity(_) => unreachable!(),
        }
    } else if na::dimension::<Vector<N>>() == 3 {
        match support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            prediction,
            &mut VoronoiSimplex3::new(),
            None,
        ) {
            GJKResult::Projection(pts, _) => ClosestPoints::WithinMargin(pts.0, pts.1),
            GJKResult::NoIntersection(_) => ClosestPoints::Disjoint,
            GJKResult::Intersection => ClosestPoints::Intersecting,
            GJKResult::Proximity(_) => unreachable!(),
        }
    } else {
        match support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            prediction,
            &mut JohnsonSimplex::new_w_tls(),
            None,
        ) {
            GJKResult::Projection(pts, _) => ClosestPoints::WithinMargin(pts.0, pts.1),
            GJKResult::NoIntersection(_) => ClosestPoints::Disjoint,
            GJKResult::Intersection => ClosestPoints::Intersecting,
            GJKResult::Proximity(_) => unreachable!(),
        }
    }
}

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut S,
    init_dir: Option<Vector<N>>,
) -> GJKResult<(Point<N>, Point<N>), Vector<N>>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let mut dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation.vector - m2.translation.vector,
        Some(dir) => dir,
    };

    if dir.is_zero() {
        dir[0] = na::one();
    }

    simplex.reset(shape::cso_support_point(m1, g1, m2, g2, dir));
    gjk::closest_points_with_max_dist(m1, g1, m2, g2, prediction, simplex)
}
