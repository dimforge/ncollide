use na::{RealField, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::query::{self, Unsupported};
use crate::shape::{Ball, Plane, Shape};
use crate::utils::IsometryOps;

/// The status of the time-of-impact computation algorithm.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TOIStatus {
    /// The TOI algorithm ran out of iterations before achieving convergence.
    ///
    /// If this happens, the content of the `NonlinearTOI` will still be a conservative approximation
    /// of the actual result so it is often fine to interpret this case as a success.
    OutOfIterations,
    /// The TOI algorithm converged successfully.
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

/// The result of a time-of-impact (TOI) computation.
#[derive(Clone, Debug)]
pub struct TOI<N: RealField> {
    /// The time at which the objects touch.
    pub toi: N,
    /// The local-space closest point on the first shape at the time of impact.
    pub witness1: Point<N>,
    /// The local-space closest point on the second shape at the time of impact.
    pub witness2: Point<N>,
    /// The local-space outward normal on the first shape at the time of impact.
    pub normal1: Unit<Vector<N>>,
    /// The local-space outward normal on the second shape at the time of impact.
    pub normal2: Unit<Vector<N>>,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: TOIStatus,
}

impl<N: RealField> TOI<N> {
    /// Swaps every data of this TOI result such that the role of both shapes are inverted.
    ///
    /// In practice, this makes it so that `self.witness1` and `self.normal1` become `self.witness2` and `self.normal2` and vice-versa.
    pub fn swapped(self) -> Self {
        Self {
            toi: self.toi,
            witness1: self.witness2,
            witness2: self.witness1,
            normal1: self.normal2,
            normal2: self.normal1,
            status: self.status,
        }
    }
}

/// Computes the smallest time at with two shapes under translational movement are separated by a
/// distance smaller or equal to `distance`.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn time_of_impact<N: RealField>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &dyn Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Result<Option<TOI<N>>, Unsupported> {
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        Ok(
            query::time_of_impact_ball_ball(&p1, vel1, b1, &p2, vel2, b2, max_toi, target_distance)
                .map(|toi| {
                    // We have to transform back the points and vectors in the sphere's local space since
                    // the time_of_impact_ball_ball did not take rotation into account.
                    TOI {
                        toi: toi.toi,
                        witness1: m1.rotation.inverse_transform_point(&toi.witness1),
                        witness2: m2.rotation.inverse_transform_point(&toi.witness2),
                        normal1: m1.inverse_transform_unit_vector(&toi.normal1),
                        normal2: m2.inverse_transform_unit_vector(&toi.normal2),
                        status: toi.status,
                    }
                }),
        )
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        Ok(query::time_of_impact_plane_support_map(
            m1,
            vel1,
            p1,
            m2,
            vel2,
            s2,
            max_toi,
            target_distance,
        ))
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        Ok(query::time_of_impact_support_map_plane(
            m1,
            vel1,
            s1,
            m2,
            vel2,
            p2,
            max_toi,
            target_distance,
        ))
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        Ok(query::time_of_impact_support_map_support_map(
            m1,
            vel1,
            s1,
            m2,
            vel2,
            s2,
            max_toi,
            target_distance,
        ))
    } else if let Some(c1) = g1.as_composite_shape() {
        Ok(query::time_of_impact_composite_shape_shape(
            m1,
            vel1,
            c1,
            m2,
            vel2,
            g2,
            max_toi,
            target_distance,
        ))
    } else if let Some(c2) = g2.as_composite_shape() {
        Ok(query::time_of_impact_shape_composite_shape(
            m1,
            vel1,
            g1,
            m2,
            vel2,
            c2,
            max_toi,
            target_distance,
        ))
    } else {
        Err(Unsupported)
    }
}
