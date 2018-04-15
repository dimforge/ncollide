use num::Zero;

use alga::linear::Translation;
use na;
use query::algorithms::gjk;
use query::algorithms::{Simplex, JohnsonSimplex, VoronoiSimplex, VoronoiSimplex};
use shape::{self, SupportMap};
use math::{Isometry, Point};

/// Distance between support-mapped shapes.
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
) -> N
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
            &mut VoronoiSimplex::new(),
            None,
        )
    } else if na::dimension::<Vector<N>>() == 3 {
        support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            &mut VoronoiSimplex::new(),
            None,
        )
    } else {
        support_map_against_support_map_with_params(
            m1,
            g1,
            m2,
            g2,
            &mut JohnsonSimplex::new_w_tls(),
            None,
        )
    }
}

/// Distance between support-mapped shapes.
///
/// This allows a more fine grained control other the underlying GJK algorigtm.
pub fn support_map_against_support_map_with_params<N, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    simplex: &mut S,
    init_dir: Option<Vector<N>>,
) -> N
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<N>,
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

    simplex.reset(*shape::cso_support_point(m1, g1, m2, g2, dir).point());

    gjk::distance(m1, g1, m2, g2, simplex)
}
