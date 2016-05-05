//! Persistant collision detection algorithms to compute contact points.
#[doc(inline)]
pub use self::contact_generator::{ContactGenerator, ContactDispatcher, ContactAlgorithm};
pub use self::default_contact_dispatcher::DefaultContactDispatcher;
pub use self::ball_ball_contact_generator::BallBallContactGenerator;
pub use self::plane_support_map_contact_generator::{PlaneSupportMapContactGenerator, SupportMapPlaneContactGenerator};
pub use self::support_map_support_map_contact_generator::SupportMapSupportMapContactGenerator;
pub use self::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use self::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use self::composite_shape_shape_contact_generator::{CompositeShapeShapeContactGenerator, ShapeCompositeShapeContactGenerator};

// FIXME: un-hide this and move everything to a folder.
#[doc(hidden)]
pub mod contact_generator;
mod default_contact_dispatcher;
mod ball_ball_contact_generator;
mod plane_support_map_contact_generator;
mod support_map_support_map_contact_generator;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod composite_shape_shape_contact_generator;
