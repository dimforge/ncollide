use na::RealField;

use crate::utils::IsometryOps;
use crate::math::{Isometry, Vector, Point, Translation};


/// A continuous rigid motion.
///
/// This is a function, assumed to be continuous, that, given a parameter `t` returns a direct isometry.
/// Mathematically speaking this is a one-parameter curve on the space of direct isometries. This curve
/// should have a continuity of at least `C0`.
pub trait RigidMotion<N: RealField> {
    /// Get a position at the time `t`.
    fn position_at_time(&self, t: N) -> Isometry<N>;
}

impl<N: RealField> RigidMotion<N> for Isometry<N> {
    fn position_at_time(&self, _: N) -> Isometry<N> {
        *self
    }
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
    /// The time at which this parametrization begins. Can be negative.
    pub t0: N,
    /// The starting isometry at `t = self.t0`.
    pub start: Isometry<N>,
    /// The translational velocity of this motion.
    pub velocity: Vector<N>,
}

impl<N: RealField> ConstantLinearVelocityRigidMotion<N> {
    /// Initialize a linear motion from a starting isometry and a translational velocity.
    pub fn new(t0: N, start: Isometry<N>, velocity: Vector<N>) -> Self {
        ConstantLinearVelocityRigidMotion {
            t0, start, velocity
        }
    }
}

impl<N: RealField> RigidMotion<N> for ConstantLinearVelocityRigidMotion<N> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        Isometry::from_parts(
            (self.start.translation.vector + self.velocity * (t - self.t0)).into(),
            self.start.rotation
        )
    }
}


/// A linear motion from a starting isometry traveling at constant translational velocity.
#[derive(Debug)]
pub struct ConstantVelocityRigidMotion<N: RealField> {
    /// The time at which this parametrization begins. Can be negative.
    pub t0: N,
    /// The starting isometry at `t = self.t0`.
    pub start: Isometry<N>,
    /// The local-space point at which the rotational part of this motion is applied.
    pub local_center: Point<N>,
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
    pub fn new(t0: N, start: Isometry<N>, local_center: Point<N>, linvel: Vector<N>, angvel: N) -> Self {
        ConstantVelocityRigidMotion {
            t0, start, local_center, linvel, angvel
        }
    }

    /// Initialize a motion from a starting isometry and linear and angular velocities.
    #[cfg(feature = "dim3")]
    pub fn new(t0: N, start: Isometry<N>, local_center: Point<N>, linvel: Vector<N>, angvel: Vector<N>) -> Self {
        ConstantVelocityRigidMotion {
            t0, start, local_center, linvel, angvel
        }
    }
}

impl<N: RealField> RigidMotion<N> for ConstantVelocityRigidMotion<N> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        let scaled_linvel = self.linvel * (t - self.t0);
        let scaled_angvel = self.angvel * (t - self.t0);

        let center = self.start.rotation * self.local_center.coords;
        let lhs = self.start.translation * Translation::from(center);
        let rhs = Translation::from(-center) * self.start.rotation;

        lhs * Isometry::new(scaled_linvel, scaled_angvel) * rhs
    }
}


/*
 * For composition.
 */

/// Trait for composing some rigid motions.
pub trait RigidMotionComposition<N: RealField>: RigidMotion<N> {
    /// Prepend a translation to the rigid motion `self`.
    fn prepend_translation(&self, translation: Vector<N>) -> PrependTranslation<N, Self> {
        PrependTranslation {
            motion: self,
            translation
        }
    }

    /// Prepend a transformation to the rigid motion `self`.
    fn prepend_transformation(&self, transformation: Isometry<N>) -> PrependTransformation<N, Self> {
        PrependTransformation {
            motion: self,
            transformation
        }
    }
}

impl<N: RealField, M: ?Sized + RigidMotion<N>> RigidMotionComposition<N> for M {}

/// The result of prepending a translation to a rigid motion.
pub struct PrependTranslation<'a, N: RealField, M: ?Sized> {
    motion: &'a M,
    translation: Vector<N>
}


impl<'a, N: RealField, M: ?Sized + RigidMotion<N>> RigidMotion<N> for PrependTranslation<'a, N, M> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        let m = self.motion.position_at_time(t);
        m * Translation::from(self.translation)
    }
}


/// The result of prepending an isometric transformation to a rigid motion.
pub struct PrependTransformation<'a, N: RealField, M: ?Sized> {
    motion: &'a M,
    transformation: Isometry<N>
}

impl<'a, N: RealField, M: ?Sized + RigidMotion<N>> RigidMotion<N> for PrependTransformation<'a, N, M> {
    fn position_at_time(&self, t: N) -> Isometry<N> {
        let m = self.motion.position_at_time(t);
        m * self.transformation
    }
}