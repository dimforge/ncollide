//! Implementation details of the `contact` and `contacts` functions.

pub use self::ball_against_ball::ball_against_ball;
pub use self::composite_shape_against_shape::{
    composite_shape_against_shape, shape_against_composite_shape,
};
pub use self::contact::{Contact, ContactPrediction, TrackedContact};
pub use self::contact_kinematic::ContactKinematic;
pub use self::contact_manifold::ContactManifold;
pub use self::plane_against_support_map::{plane_against_support_map, support_map_against_plane};
pub use self::shape_against_shape::shape_against_shape as contact_internal;
pub use self::support_map_against_support_map::{
    support_map_against_support_map, support_map_against_support_map_with_dir,
    support_map_against_support_map_with_simplex,
};

mod ball_against_ball;
mod composite_shape_against_shape;
mod contact;
mod contact_kinematic;
mod contact_manifold;
mod plane_against_support_map;
mod shape_against_shape;
mod support_map_against_support_map;
