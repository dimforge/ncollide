//! Volume and inertia tensor computation.

pub use volumetric::volumetric::{Volumetric, InertiaTensor};

#[cfg(not(feature = "4d"))]
pub use volumetric::volumetric_ball::ball_volume;
#[cfg(not(feature = "4d"))]
pub use volumetric::volumetric_cuboid::cuboid_volume;
#[cfg(not(feature = "4d"))]
pub use volumetric::volumetric_cone::cone_volume;
#[cfg(not(feature = "4d"))]
pub use volumetric::volumetric_capsule::capsule_volume;
#[cfg(not(feature = "4d"))]
pub use volumetric::volumetric_cylinder::cylinder_volume;

pub mod volumetric;
#[cfg(not(feature = "4d"))]
mod volumetric_ball;
#[cfg(not(feature = "4d"))]
mod volumetric_cylinder;
#[cfg(not(feature = "4d"))]
mod volumetric_cuboid;
#[cfg(not(feature = "4d"))]
mod volumetric_cone;
#[cfg(not(feature = "4d"))]
mod volumetric_capsule;
mod volumetric_compound;
#[cfg(not(feature = "4d"))]
mod volumetric_convex;
