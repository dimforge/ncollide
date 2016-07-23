//! Persistent collision detection algorithms to compute contact points.

use na::{Point2, Point3, Isometry2, Isometry3};
pub use self::narrow_phase::{NarrowPhase, ContactPairs, Contacts, ProximityPairs};
pub use self::default_narrow_phase::DefaultNarrowPhase;

#[doc(inline)]
pub use self::contact_generator::{
    ContactGenerator,
    ContactDispatcher,
    ContactAlgorithm,
    DefaultContactDispatcher,
    BallBallContactGenerator,
    PlaneSupportMapContactGenerator, SupportMapPlaneContactGenerator,
    PlaneSupportMapContactManifoldGenerator, SupportMapPlaneContactManifoldGenerator,
    SupportMapSupportMapContactGenerator,
    CompositeShapeShapeContactGenerator, ShapeCompositeShapeContactGenerator,
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
pub use self::contact_signal::{ContactSignal, ContactHandler};
#[doc(inline)]
pub use self::proximity_signal::{ProximitySignal, ProximityHandler};
//
// use na::{Point2, Point3, Vector2, Vector3, Isometry2, Isometry3};

// FIXME: un-hide this and move everything to a folder.
#[doc(hidden)]
pub mod contact_generator;
#[doc(hidden)]
pub mod proximity_detector;
#[doc(hidden)]
pub mod narrow_phase;
mod default_narrow_phase;
#[doc(hidden)]
pub mod contact_signal;
#[doc(hidden)]
pub mod proximity_signal;

/// Trait-object for 2-dimensional contact generation.
pub type ContactAlgorithm2<N> = ContactAlgorithm<Point2<N>, Isometry2<N>>;
/// Trait-object for 3-dimensional contact generation.
pub type ContactAlgorithm3<N> = ContactAlgorithm<Point3<N>, Isometry3<N>>;
