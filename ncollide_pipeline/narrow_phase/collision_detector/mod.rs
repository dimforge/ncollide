//! Persistant collision detection algorithms to compute contact points.
#[doc(inline)]
pub use self::collision_detector::{CollisionDetector, CollisionDispatcher, CollisionAlgorithm};
pub use self::default_collision_dispatcher::DefaultCollisionDispatcher;
pub use self::ball_ball_collision_detector::BallBallCollisionDetector;
pub use self::plane_support_map_collision_detector::{PlaneSupportMapCollisionDetector, SupportMapPlaneCollisionDetector};
pub use self::support_map_support_map_collision_detector::SupportMapSupportMapCollisionDetector;
pub use self::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use self::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use self::composite_shape_repr_collision_detector::{CompositeShapeShapeCollisionDetector, ShapeCompositeShapeCollisionDetector};

// FIXME: un-hide this and move everything to a folder.
#[doc(hidden)]
pub mod collision_detector;
mod default_collision_dispatcher;
mod ball_ball_collision_detector;
mod plane_support_map_collision_detector;
mod support_map_support_map_collision_detector;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod composite_shape_repr_collision_detector;
