//! Interpolation of the motion of an object.

pub use self::rigid_motion::{RigidMotion, InterpolatedRigidMotion,
                             ConstantLinearVelocityRigidMotion, ConstantVelocityRigidMotion};

mod rigid_motion;