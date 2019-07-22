use na::RealField;

use crate::utils::IsometryOps;
use crate::math::{Isometry, Vector, Rotation};


/// A continuous interpolation of isometries.
pub trait RigidMotion<N: RealField> {
    /// Get a position at the time `t`.
    fn position_at_time(&self, t: N) -> Isometry<N>;
}


/// Interpolation between two isometries using LERP for the translation part and SLERP for the rotation.
pub struct InterpolatedRigidMotion<N: RealField> {
    /// The transformation at `t = 0.0`.
    pub start: Isometry<N>,
    /// The transformation at `t = 1.0`.
    pub end: Isometry<N>,
}

impl<N: RealField> InterpolatedRigidMotion<N> {
    /// Initialize a lerp-slerp interpolation with the given start and end transformations.
    ///
    /// The `start` is the transformation at the time `t = 0.0` and `end` is the transformation at
    /// the time `t = 1.0`.
    pub fn new(start: Isometry<N>, end: Isometry<N>) -> Self {
        InterpolatedRigidMotion {
            start, end
        }
    }
}

impl<N: RealField> RigidMotion<N> for InterpolatedRigidMotion<N> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        self.start.lerp_slerp(&self.end, t)
    }
}

/// A linear motion from a starting isometry traveling at constant translational velocity.
pub struct ConstantLinearVelocityRigidMotion<N: RealField> {
    /// The starting isometry at `t = 0`.
    pub start: Isometry<N>,
    /// The translational velocity of this motion.
    pub velocity: Vector<N>,
}

impl<N: RealField> ConstantLinearVelocityRigidMotion<N> {
    /// Initialize a linear motion frow a starting isometry and a translational velocity.
    pub fn new(start: Isometry<N>, velocity: Vector<N>) -> Self {
        ConstantLinearVelocityRigidMotion {
            start, velocity
        }
    }
}

impl<N: RealField> RigidMotion<N> for ConstantLinearVelocityRigidMotion<N> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        Isometry::from_parts(
            (self.start.translation.vector + self.velocity * t).into(),
            self.start.rotation
        )
    }
}


/// A linear motion from a starting isometry traveling at constant translational velocity.
pub struct ConstantVelocityRigidMotion<N: RealField> {
    /// The time at which this parametrization begins. Can be negative.
    pub t0: N,
    /// The starting isometry at `t = self.start_t`.
    pub start: Isometry<N>,
    /// The translational velocity of this motion.
    pub linvel: Vector<N>,
    /// The angular velocity of this motion.
    #[cfg(feature = "dim2")]
    pub angvel: N,
    /// The angular velocity of this motion.
    #[cfg(feature = "dim3")]
    pub angvel: Vector<N>,

}

impl<N: RealField> ConstantVelocityRigidMotion<N> {
    /// Initialize a motion from a starting isometry and linear and angular velocities.
    #[cfg(feature = "dim2")]
    pub fn new(t0: N, start: Isometry<N>, linvel: Vector<N>, angvel: N) -> Self {
        ConstantVelocityRigidMotion {
            t0, start, linvel, angvel
        }
    }

    /// Initialize a motion from a starting isometry and linear and angular velocities.
    #[cfg(feature = "dim3")]
    pub fn new(t0: N, start: Isometry<N>, linvel: Vector<N>, angvel: Vector<N>) -> Self {
        ConstantVelocityRigidMotion {
            t0, start, linvel, angvel
        }
    }
}

impl<N: RealField> RigidMotion<N> for ConstantVelocityRigidMotion<N> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        Isometry::from_parts(
            (self.start.translation.vector + self.linvel * (t - self.t0)).into(),
            Rotation::new(self.angvel * (t - self.t0)) * self.start.rotation
        )
    }
}