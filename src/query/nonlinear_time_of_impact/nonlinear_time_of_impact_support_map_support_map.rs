use na::RealField;

use crate::math::{Isometry, Vector};
use crate::query::{self, ClosestPoints};
use crate::shape::SupportMap;

/// Time of impacts between two support-mapped shapes under a rigid motion.
pub fn nonlinear_time_of_impact_support_map_support_map<N, G1: ?Sized, G2: ?Sized>(
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
    unimplemented!()
    /*
    let motion = RigidBodyMotion::new();
    let mut toi = N::zero();

    loop {
        let pos1 = motion.iterpolate(toi);
        match query::closest_points(&pos1, g1, m2, g2, N::max_value()) {
            ClosestPoints::Intersecting => return None,
            ClosestPoints::WithinMargin(p1, p2) => {
                if let Some((dir, dist)) = Unit::try_new_and_get(p2 - p1, N::default_epsilon()) {
                    let cross = motion.angular.cross(dir);
                    let pt = g1.support_point(&pos1, &bitangent);
                    let mu = cross.dot(&(pt - pos1.translation.vector())) + dir.dot(&motion.linear);
                    toi += dist / mu;
                } else {
                    return None;
                }
            }
        }
    }
    */
}
