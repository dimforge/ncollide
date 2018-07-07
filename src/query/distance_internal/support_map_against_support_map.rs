use math::{Isometry, Vector};
use na::{self, Real, Unit};
use query::algorithms::VoronoiSimplex;
use query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use shape::SupportMap;

/// Distance between support-mapped shapes.
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
) -> N
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    support_map_against_support_map_with_params(m1, g1, m2, g2, &mut VoronoiSimplex::new(), None)
}

/// Distance between support-mapped shapes.
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    simplex: &mut VoronoiSimplex<N>,
    init_dir: Option<Vector<N>>,
) -> N
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation.vector - m2.translation.vector,
        Some(dir) => dir,
    };

    let m12 = m1.inverse() * m2;
    if let Some(dir) = Unit::try_new(dir, N::default_epsilon()) {
        simplex.reset(CSOPoint::from_shapes_local1(g1, &m12, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes_local1(
            g1,
            &m12,
            g2,
            &Vector::x_axis(),
        ));
    }

    match gjk::closest_points(g1, &m12, g2, N::max_value(), true, simplex) {
        GJKResult::Intersection => N::zero(),
        GJKResult::ClosestPoints(p1, p2, _) => na::distance(&p1, &p2),
        GJKResult::Proximity(_) => unreachable!(),
        GJKResult::NoIntersection(_) => N::zero(), // FIXME: GJK did not converge.
    }
}
