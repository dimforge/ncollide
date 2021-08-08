use na::{self, RealField, Unit};

use crate::math::{Isometry, Vector};
use crate::query::algorithms::VoronoiSimplex;
use crate::query::algorithms::{gjk, gjk::GJKResult, CSOPoint};
use crate::query::Proximity;
use crate::shape::SupportMap;

/// Proximity between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
pub fn proximity_support_map_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
) -> Proximity
where
    N: RealField + Copy,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    proximity_support_map_support_map_with_params(
        m1,
        g1,
        m2,
        g2,
        margin,
        &mut VoronoiSimplex::new(),
        None,
    )
    .0
}

/// Proximity between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn proximity_support_map_support_map_with_params<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
    simplex: &mut VoronoiSimplex<N>,
    init_dir: Option<Unit<Vector<N>>>,
) -> (Proximity, Unit<Vector<N>>)
where
    N: RealField + Copy,
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

    simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &dir));

    match gjk::closest_points(m1, g1, m2, g2, margin, false, simplex) {
        GJKResult::Intersection => (Proximity::Intersecting, dir),
        GJKResult::Proximity(dir) => (Proximity::WithinMargin, dir),
        GJKResult::NoIntersection(dir) => (Proximity::Disjoint, dir),
        GJKResult::ClosestPoints(..) => unreachable!(),
    }
}
