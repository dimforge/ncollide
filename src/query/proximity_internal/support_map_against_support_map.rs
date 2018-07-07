use na::{self, Real, Unit};

use math::{Isometry, Vector};
use query::algorithms::VoronoiSimplex;
use query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use query::Proximity;
use shape::SupportMap;

/// Proximity between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
) -> Proximity
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    support_map_against_support_map_with_params(
        m1,
        g1,
        m2,
        g2,
        margin,
        &mut VoronoiSimplex::new(),
        None,
    ).0
}

/// Proximity between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
    simplex: &mut VoronoiSimplex<N>,
    init_dir: Option<Unit<Vector<N>>>,
) -> (Proximity, Unit<Vector<N>>)
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or zero."
    );

    let dir = if let Some(init_dir) = init_dir {
        init_dir
    } else if let Some(init_dir) = Unit::try_new(
        m2.translation.vector - m1.translation.vector,
        N::default_epsilon(),
    ) {
        init_dir
    } else {
        Vector::x_axis()
    };

    let m12 = m1.inverse() * m2;
    simplex.reset(CSOPoint::from_shapes_local1(g1, &m12, g2, &dir));

    match gjk::closest_points(g1, &m12, g2, margin, false, simplex) {
        GJKResult::Intersection => (Proximity::Intersecting, dir),
        GJKResult::Proximity(dir) => (Proximity::WithinMargin, dir),
        GJKResult::NoIntersection(dir) => (Proximity::Disjoint, dir),
        GJKResult::ClosestPoints(..) => unreachable!(),
    }
}
