//! Interpolation of the motion of an object.

pub use self::rigid_motion::{
    ConstantLinearVelocityRigidMotion, ConstantVelocityRigidMotion, InterpolatedRigidMotion,
    RigidMotion, RigidMotionComposition,
};

mod rigid_motion;
