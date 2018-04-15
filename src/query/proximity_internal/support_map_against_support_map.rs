use alga::linear::Translation;
use na;

use shape::{self, AnnotatedPoint, SupportMap};
use query::algorithms::gjk;
use query::algorithms::{Simplex, JohnsonSimplex, VoronoiSimplex, VoronoiSimplex};
use query::Proximity;
use math::{Isometry, Point};

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
    M: Isometry<P>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    if na::dimension::<Vector<N>>() == 2 {
        support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            margin,
            &mut VoronoiSimplex::new(),
            None,
        ).0
    } else if na::dimension::<Vector<N>>() == 3 {
        support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            margin,
            &mut VoronoiSimplex::new(),
            None,
        ).0
    } else {
        support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            margin,
            &mut JohnsonSimplex::new_w_tls(),
            None,
        ).0
    }
}

/// Proximity between support-mapped shapes (`Cuboid`, `ConvexHull`, etc.)
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
    simplex: &mut S,
    init_dir: Option<Vector<N>>,
) -> (Proximity, Vector<N>)
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let mut dir = match init_dir {
        // FIXME: or m2.translation - m1.translation ?
        None => m1.translation.vector - m2.translation.vector,
        Some(dir) => dir,
    };

    if dir == na::zero() {
        dir[0] = na::one();
    }

    simplex.reset(shape::cso_support_point(m1, g1, m2, g2, dir));

    gjk::proximity(m1, g1, m2, g2, margin, simplex)
}
