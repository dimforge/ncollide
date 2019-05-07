use na::RealField;

use crate::math::{Isometry, Point, Vector};
use crate::query;
use crate::shape::{Ball, Plane, Shape};

/// Computes the smallest time of impact of two shapes under translational movement.
///
/// Returns `0.0` if the objects are touching or penetrating.
pub fn nonlinear_time_of_impact<N: RealField>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N>
{
    if let (Some(b1), Some(b2)) = (g1.as_shape::<Ball<N>>(), g2.as_shape::<Ball<N>>()) {
        let p1 = Point::from(m1.translation.vector);
        let p2 = Point::from(m2.translation.vector);

        query::nonlinear_time_of_impact_ball_ball(&p1, vel1, b1, &p2, vel2, b2)
    } else if let (Some(p1), Some(s2)) = (g1.as_shape::<Plane<N>>(), g2.as_support_map()) {
        query::nonlinear_time_of_impact_plane_support_map(m1, vel1, p1, m2, vel2, s2)
    } else if let (Some(s1), Some(p2)) = (g1.as_support_map(), g2.as_shape::<Plane<N>>()) {
        query::nonlinear_time_of_impact_support_map_plane(m1, vel1, s1, m2, vel2, p2)
    } else if let (Some(s1), Some(s2)) = (g1.as_support_map(), g2.as_support_map()) {
        query::nonlinear_time_of_impact_support_map_support_map(m1, vel1, s1, m2, vel2, s2)
    } else if let Some(c1) = g1.as_composite_shape() {
        query::nonlinear_time_of_impact_composite_shape_shape(m1, vel1, c1, m2, vel2, g2)
    } else if let Some(c2) = g2.as_composite_shape() {
        query::nonlinear_time_of_impact_shape_composite_shape(m1, vel1, g1, m2, vel2, c2)
    } else {
        panic!("No algorithm known to compute a contact point between the given pair of shapes.")
    }
}

struct RigidBodyMotion<'a> {
    start: &'a Isometry<N>,
    linvel: &'a Vector<N>,
    #[cfg(feature = "dim3")]
    angvel: &'a Vector<N>,
    #[cfg(feature = "dim2")]
    angvel: N
}

impl<'a> RigidBodyMotion<'a> {
    #[cfg(feature = "dim3")]
    fn new(start: &'a Isometry<N>, linvel: &'a Vector<N>, angvel: &'a Vector<N>) -> Self {
        Self {
            start, linvel, angvel
        }
    }

    #[cfg(feature = "dim2")]
    fn new(start: &'a Isometry<N>, linvel: &'a Vector<N>, angvel: N) -> Self {
        Self {
            start, linvel, angvel
        }
    }

    fn interpolate(&self, t: N) -> Isometry<N> {
        let dpos = Isometry::new(self.linvel * t, self.angvel * t);
        dpos * *self.start
    }
}