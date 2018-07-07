use math::{Isometry, Vector};
use na::{Real, Unit};
use query::algorithms::VoronoiSimplex;
use query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use query::ClosestPoints;
use shape::SupportMap;

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    g1: &G1,
    m12: &Isometry<N>,
    g2: &G2,
    prediction: N,
) -> ClosestPoints<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    match support_map_against_support_map_with_params(
        g1,
        m12,
        g2,
        prediction,
        &mut VoronoiSimplex::new(),
        None,
    ) {
        GJKResult::ClosestPoints(pt1, pt2, _) => ClosestPoints::WithinMargin(pt1, pt2),
        GJKResult::NoIntersection(_) => ClosestPoints::Disjoint,
        GJKResult::Intersection => ClosestPoints::Intersecting,
        GJKResult::Proximity(_) => unreachable!(),
    }
}

/// Closest points between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, G1: ?Sized, G2: ?Sized>(
    g1: &G1,
    m12: &Isometry<N>,
    g2: &G2,
    prediction: N,
    simplex: &mut VoronoiSimplex<N>,
    init_dir: Option<Vector<N>>,
) -> GJKResult<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dir = match init_dir {
        // FIXME: or m12.translation ?
        None => -m12.translation.vector,
        Some(dir) => dir,
    };

    if let Some(dir) = Unit::try_new(dir, N::default_epsilon()) {
        simplex.reset(CSOPoint::from_shapes_local1(g1, m12, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes_local1(g1, m12, g2, &Vector::x_axis()));
    }

    gjk::closest_points(g1, m12, g2, prediction, true, simplex)
}
