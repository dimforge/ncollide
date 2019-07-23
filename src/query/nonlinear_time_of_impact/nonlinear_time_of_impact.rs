use na::RealField;

use crate::interpolation::RigidMotion;
use crate::math::{Isometry, Point, Vector};
use crate::query;
use crate::shape::{Ball, Plane, Shape};

/// Computes the smallest time of impact of two shapes under translational movement.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn nonlinear_time_of_impact<N: RealField>(
    motion1: &(impl RigidMotion<N> + ?Sized),
    g1: &Shape<N>,
    motion2: &(impl RigidMotion<N> + ?Sized),
    g2: &Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Option<NonlinearTOI<N>>
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        query::nonlinear_time_of_impact_ball_ball(motion1, b1, motion2, b2, max_toi, target_distance)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::nonlinear_time_of_impact_support_map_support_map(motion1, s1, motion2, s2, max_toi, target_distance)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::nonlinear_time_of_impact_composite_shape_shape(motion1, c1, motion2, g2, max_toi, target_distance)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::nonlinear_time_of_impact_shape_composite_shape(motion1, g1, motion2, c2, max_toi, target_distance)
    /* } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
//        query::nonlinear_time_of_impact_plane_support_map(m1, vel1, p1, m2, vel2, s2)
        unimplemented!()
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
//        query::nonlinear_time_of_impact_support_map_plane(m1, vel1, s1, m2, vel2, p2)
        unimplemented!() */
    } else {
        eprintln!("No algorithm known to compute a contact point between the given pair of shapes.");
        None
    }
}

/// The status of the time-of-impact computation algorithm.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum NonlinearTOIStatus {
    /// The nonlinear TOI algorithm ran out of iterations before achieving convergence.
    ///
    /// If this happens, the content of the `NonlinearTOI` will still be a conservative approximation
    /// of the actual result so it is often fine to interpret this case as a success.
    OutOfIterations,
    /// The nonlinear TOI algorithm converged successfully.
    Converged,
    /// Something went wrong during the TOI computation, likely due to numerical instabilities.
    ///
    /// If this happens, the content of the `NonlinearTOI` will still be a conservative approximation
    /// of the actual result so it is often fine to interpret this case as a success.
    Failed,
    /// The two shape already overlap at the time 0.
    ///
    /// If this happens, the witness points provided by the `NonlinearTOI` will be invalid.
    Penetrating,
}

/// The result of a nonlinear time-of-impact (TOI) computation.
#[derive(Clone, Debug)]
pub struct NonlinearTOI<N: RealField> {
    /// The time at which the objects touch.
    pub toi: N,
    /// The local-space closest point on the first shape at the time of impact.
    pub witness1: Point<N>,
    /// The local-space closest point on the second shape at the time of impact.
    pub witness2: Point<N>,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: NonlinearTOIStatus
}