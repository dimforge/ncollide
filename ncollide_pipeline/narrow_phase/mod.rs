//! Collision detection algorithms and structure for the Narrow Phase.
#[doc(inline)]
pub use self::collision_detector::{CollisionDetector, CollisionDispatcher, CollisionAlgorithm};
pub use self::ball_ball::BallBall;
pub use self::plane_support_map::{PlaneSupportMap, SupportMapPlane};
pub use self::support_map_support_map::SupportMapSupportMap;
pub use self::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use self::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use self::composite_shape_repr::{CompositeShapeRepr, ReprCompositeShape};
pub use self::basic_collision_dispatcher::BasicCollisionDispatcher;
#[doc(inline)]
pub use self::contact_signal::{ContactSignal, ContactSignalHandler};
//
// use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

#[doc(hidden)]
pub mod collision_detector;
#[doc(hidden)]
pub mod contact_signal;
mod ball_ball;
mod plane_support_map;
mod support_map_support_map;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod composite_shape_repr;
mod basic_collision_dispatcher;
