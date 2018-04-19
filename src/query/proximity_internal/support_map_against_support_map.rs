use na::{self, Real, Unit};

use shape::{self, SupportMap};
use query::algorithms::{gjk, CSOPoint, gjk::GJKResult};
use query::algorithms::{Simplex, VoronoiSimplex};
use query::Proximity;
use math::{Isometry, Point, Vector};

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
    init_dir: Option<Vector<N>>,
) -> (Proximity, Vector<N>)
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation.vector - m2.translation.vector,
        Some(dir) => dir,
    };

    if let Some(dir) = Unit::try_new(dir, N::default_epsilon()) {
        simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &dir));
    } else {
        simplex.reset(CSOPoint::from_shapes(m1, g1, m2, g2, &Vector::x_axis()));
    }

    match gjk::closest_points(
        m1,
        g1,
        m2,
        g2,
        margin,
        false,
        simplex,
    ) {
        GJKResult::Intersection => (Proximity::Intersecting, dir),
        GJKResult::Proximity(dir) => (Proximity::WithinMargin, dir),
        GJKResult::NoIntersection(dir) => (Proximity::Disjoint, dir),
        GJKResult::ClosestPoints(..) => unreachable!()
    }
}
