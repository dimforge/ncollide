//! Persistent collision detection algorithms to compute contact points.

use na::{Isometry2, Isometry3, Point2, Point3};
pub use self::narrow_phase::{ContactPairs, Contacts, NarrowPhase, ProximityPairs};
pub use self::default_narrow_phase::DefaultNarrowPhase;

#[doc(inline)]
pub use self::contact_generator::{BallBallContactGenerator, CompositeShapeShapeContactGenerator,
                                  ContactAlgorithm, ContactDispatcher, ContactGenerator,
                                  DefaultContactDispatcher, IncrementalContactManifoldGenerator,
                                  OneShotContactManifoldGenerator,
                                  PlaneSupportMapContactGenerator,
                                  ShapeCompositeShapeContactGenerator,
                                  SupportMapPlaneContactGenerator,
                                  SupportMapSupportMapContactGenerator};

#[doc(inline)]
pub use self::proximity_detector::{BallBallProximityDetector,
                                   CompositeShapeShapeProximityDetector,
                                   DefaultProximityDispatcher, PlaneSupportMapProximityDetector,
                                   ProximityAlgorithm, ProximityDetector, ProximityDispatcher,
                                   ShapeCompositeShapeProximityDetector,
                                   SupportMapPlaneProximityDetector,
                                   SupportMapSupportMapProximityDetector};

#[doc(hidden)]
pub mod contact_generator;
#[doc(hidden)]
pub mod proximity_detector;
#[doc(hidden)]
pub mod narrow_phase;
mod default_narrow_phase;

/// Trait-object for 2-dimensional contact generation.
pub type ContactAlgorithm2<N> = ContactAlgorithm<Point2<N>, Isometry2<N>>;
/// Trait-object for 3-dimensional contact generation.
pub type ContactAlgorithm3<N> = ContactAlgorithm<Point3<N>, Isometry3<N>>;
