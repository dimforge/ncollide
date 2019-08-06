use na::{RealField, Unit};

use crate::math::{Isometry, Vector, Point};
use crate::query::{self, TOI, TOIStatus};
use crate::query::algorithms::{VoronoiSimplex, special_support_maps::DilatedShape};
use crate::query::algorithms::gjk::{self, GJKResult};
use crate::shape::SupportMap;
use crate::utils::IsometryOps;

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
            .and_then(|(toi, normal, witness1, witness2)|
                if toi > max_toi {
                    None
                } else {
                    Some(TOI {
                        toi,
                        normal1: Unit::new_unchecked(m1.inverse_transform_vector(&normal)),
                        normal2: Unit::new_unchecked(m2.inverse_transform_vector(&-normal)),
                        witness1: m1.inverse_transform_point(&witness1),
                        witness2: m2.inverse_transform_point(&witness2),
                        status: if toi.is_zero() { TOIStatus::Penetrating } else { TOIStatus::Converged }, // XXX
                    })
                }
            )
    } else {
        let dilated1 = DilatedShape {
            shape: g1,
            radius: target_distance
        };

        gjk::directional_distance(m1, &dilated1, m2, g2, &dvel, &mut VoronoiSimplex::new())
            .and_then(|(toi, normal, witness1, witness2)|
                if toi > max_toi {
                    None
                } else {
                    // This is mutable because, if the TOI is zero, we have to determine
                    // if the status isn't really a TOIStatus::Penetrating.
                    let mut status = TOIStatus::Converged;

                    if toi.is_zero() {
                        // The TOI is zero but we don't have valid witness points and normal
                        // yet because of we based our computations so far on the dilated shape.
                        // Therefore we need an extra step to retrieve the actual closest points, and
                        // determine if the actual shapes are penetrating or not.
                        // FIXME: all those computations are costly. Add a variant that returns only
                        // the TOI and does not computes the normal and witness points?
                        match query::closest_points_support_map_support_map_with_params(
                            m1,
                            g1,
                            m2,
                            g2,
                            target_distance,
                            &mut VoronoiSimplex::new(),
                            Some(normal),
                        ) {
                            GJKResult::ClosestPoints(pt1, pt2, _) => {
                                // Ok, we managed to compute the witness points.
                                let normal = Unit::new_normalize(pt2 - pt1);
                                let normal1 = Unit::new_unchecked(m1.inverse_transform_vector(&normal));
                                return Some(TOI {
                                    toi,
                                    normal1,
                                    normal2: Unit::new_unchecked(m2.inverse_transform_vector(&-normal)),
                                    witness1: m1.inverse_transform_point(&witness1),
                                    witness2: m2.inverse_transform_point(&witness2),
                                    status: TOIStatus::Converged,
                                })
                            },
                            GJKResult::NoIntersection(_) => {
                                // This should never happen.
                            },
                            GJKResult::Intersection => status = TOIStatus::Penetrating,
                            GJKResult::Proximity(_) => unreachable!(),
                        }
                    }

                    let normal1 = Unit::new_unchecked(m1.inverse_transform_vector(&normal));
                    Some(TOI {
                        toi,
                        normal1,
                        normal2: Unit::new_unchecked(m2.inverse_transform_vector(&-normal)),
                        witness1: m1.inverse_transform_point(&witness1) - *normal1 * target_distance,
                        witness2: m2.inverse_transform_point(&witness2),
                        status,
                    })
                }
            )
    }
}
