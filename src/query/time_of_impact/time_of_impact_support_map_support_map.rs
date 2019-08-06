use na::RealField;

use crate::math::{Isometry, Vector, Point};
use crate::query::{TOI, TOIStatus};
use crate::query::algorithms::{gjk, VoronoiSimplex, special_support_maps::DilatedShape};
use crate::shape::SupportMap;

/// Time of impacts between two support-mapped shapes under translational movement.
pub fn time_of_impact_support_map_support_map<N, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let dvel = vel2 - vel1;

    if target_distance.is_zero() {
        gjk::directional_distance(m1, g1, m2, g2, &dvel, &mut VoronoiSimplex::new())
            .and_then(|(toi, _)|
                if toi > max_toi {
                    None
                } else {
                    Some(TOI {
                        toi,
                        normal1: Vector::x_axis(),
                        normal2: Vector::x_axis(),
                        witness1: Point::origin(), // XXX
                        witness2: Point::origin(), // XXX
                        status: TOIStatus::Converged, // XXX
                    })
                }
            )
    } else {
        let dilated1 = DilatedShape {
            shape: g1,
            radius: target_distance
        };

        gjk::directional_distance(m1, &dilated1, m2, g2, &dvel, &mut VoronoiSimplex::new())
            .and_then(|(toi, _)|
                if toi > max_toi {
                    None
                } else {
                    Some(TOI {
                        toi,
                        normal1: Vector::x_axis(),
                        normal2: Vector::x_axis(),
                        witness1: Point::origin(), // XXX
                        witness2: Point::origin(), // XXX
                        status: TOIStatus::Converged, // XXX
                    })
                }
            )
    }
}
