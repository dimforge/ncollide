//! Volume and inertia tensor computation.

pub use volumetric::volumetric::{Volumetric, InertiaTensor};
pub use volumetric::volumetric_ball::ball_volume;
pub use volumetric::volumetric_cuboid::cuboid_volume;

// #[dim3]
pub use volumetric::volumetric_cone::cone_volume;
// #[dim3]
pub use volumetric::volumetric_capsule::capsule_volume;
// #[dim3]
pub use volumetric::volumetric_cylinder::cylinder_volume;

// #[dim2]
pub use volumetric::volumetric_cone::cone_volume;
// #[dim2]
pub use volumetric::volumetric_capsule::capsule_volume;
// #[dim2]
pub use volumetric::volumetric_cylinder::cylinder_volume;

pub mod volumetric;
mod volumetric_ball;
mod volumetric_cylinder;
mod volumetric_cuboid;
mod volumetric_cone;
mod volumetric_capsule;
mod volumetric_compound;
