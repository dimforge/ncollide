use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::algorithms::{gjk, VoronoiSimplex};
use crate::shape::SupportMap;

/// Time of impacts between two support-mapped shapes under translational movement.
pub fn time_of_impact_support_map_support_map<N, G1: ?Sized, G2: ?Sized>(
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
