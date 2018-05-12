use na::Real;

use math::{Isometry, Vector};
use query::algorithms::{gjk, VoronoiSimplex};
use shape::SupportMap;

/// Time of impacts between two support-mapped shapes under translational movement.
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: Real,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dvel = vel1 - vel2;
    gjk::directional_distance(m1, g1, m2, g2, &dvel, &mut VoronoiSimplex::new())
}
