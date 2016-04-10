//! Persistant proximity detection algorithms.

pub use self::proximity_detector::{ProximityDetector, ProximityDispatcher, ProximityAlgorithm};
pub use self::ball_ball_proximity_detector::BallBallProximityDetector;
pub use self::plane_support_map_proximity_detector::{PlaneSupportMapProximityDetector, SupportMapPlaneProximityDetector};
pub use self::support_map_support_map_proximity_detector::SupportMapSupportMapProximityDetector;
pub use self::composite_shape_repr_proximity_detector::{CompositeShapeReprProximityDetector, ReprCompositeShapeProximityDetector};
pub use self::default_proximity_dispatcher::DefaultProximityDispatcher;

#[doc(hidden)]
pub mod proximity_detector;
mod ball_ball_proximity_detector;
mod plane_support_map_proximity_detector;
mod support_map_support_map_proximity_detector;
mod composite_shape_repr_proximity_detector;
mod default_proximity_dispatcher;
