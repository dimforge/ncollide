//! Persistant collision detection algorithms to compute contact points.
pub use self::narrow_phase::{NarrowPhase, ContactPairs, Contacts};
pub use self::default_narrow_phase::DefaultNarrowPhase;

#[doc(inline)]
pub use self::collision_detector::{
    CollisionDetector,
    CollisionDispatcher,
    CollisionAlgorithm,
    DefaultCollisionDispatcher,
    BallBallCollisionDetector,
    PlaneSupportMapCollisionDetector, SupportMapPlaneCollisionDetector,
    SupportMapSupportMapCollisionDetector,
    CompositeShapeShapeCollisionDetector, ShapeCompositeShapeCollisionDetector,
    IncrementalContactManifoldGenerator,
    OneShotContactManifoldGenerator
};

#[doc(inline)]
pub use self::proximity_detector::{
    ProximityDetector,
    ProximityDispatcher,
    ProximityAlgorithm,
    DefaultProximityDispatcher,
    BallBallProximityDetector,
    PlaneSupportMapProximityDetector, SupportMapPlaneProximityDetector,
    SupportMapSupportMapProximityDetector,
    CompositeShapeShapeProximityDetector,
    ShapeCompositeShapeProximityDetector
};

#[doc(inline)]
pub use self::contact_signal::{ContactSignal, ContactSignalHandler};
#[doc(inline)]
pub use self::proximity_signal::{ProximitySignal, ProximitySignalHandler};
//
// use na::{Point2, Point3, Vector2, Vector3, Isometry2, Isometry3};

// FIXME: un-hide this and move everything to a folder.
#[doc(hidden)]
pub mod collision_detector;
#[doc(hidden)]
pub mod proximity_detector;
#[doc(hidden)]
pub mod narrow_phase;
mod default_narrow_phase;
#[doc(hidden)]
pub mod contact_signal;
#[doc(hidden)]
pub mod proximity_signal;
