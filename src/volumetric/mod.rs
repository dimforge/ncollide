//! Volume and inertia tensor computation.

pub use volumetric::volumetric::{Volumetric, InertiaTensor};
pub use volumetric::volumetric_ball::ball_volume;
pub use volumetric::volumetric_box::box_volume;

#[cfg(dim3)]
pub use volumetric::volumetric_cone::cone_volume;
#[cfg(dim3)]
pub use volumetric::volumetric_capsule::capsule_volume;
#[cfg(dim3)]
pub use volumetric::volumetric_cylinder::cylinder_volume;

#[cfg(dim2)]
pub use volumetric::volumetric_cone::cone_volume;
#[cfg(dim2)]
pub use volumetric::volumetric_capsule::capsule_volume;
#[cfg(dim2)]
pub use volumetric::volumetric_cylinder::cylinder_volume;

mod volumetric;
mod volumetric_ball;
mod volumetric_cylinder;
mod volumetric_box;
mod volumetric_cone;
mod volumetric_capsule;
mod volumetric_compound;
mod volumetric_convex;
mod volumetric_plane;
mod volumetric_mesh;
mod volumetric_triangle;
mod volumetric_segment;
