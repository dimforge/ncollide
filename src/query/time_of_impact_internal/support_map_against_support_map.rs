use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::algorithms::{gjk, VoronoiSimplex};
use crate::shape::SupportMap;

/// Time of impact between two support-mapped shapes under translational movement.
pub fn support_map_against_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dvel = vel2 - vel1;
    gjk::directional_distance(m1, g1, m2, g2, &dvel, &mut VoronoiSimplex::new())
}

/// Time of impact and contact normal between two support-mapped shapes under
/// translational movement.
pub fn support_map_against_support_map_with_normal<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dvel = vel2 - vel1;
    gjk::directional_distance_and_normal(m1, g1, m2, g2, &dvel, &mut VoronoiSimplex::new())
}
