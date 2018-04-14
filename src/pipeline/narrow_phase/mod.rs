//! Persistent collision detection algorithms to compute contact points.

pub use self::narrow_phase::{ContactManifolds, ContactPairs, NarrowPhase, ProximityPairs};
pub use self::default_narrow_phase::DefaultNarrowPhase;

#[doc(inline)]
pub use self::contact_generator::{BallBallManifoldGenerator,
                                //   BallConvexPolyhedronManifoldGenerator,
                                //   CompositeShapeShapeManifoldGenerator,
                                  ContactAlgorithm,
                                  ContactDispatcher,
                                  ContactManifoldGenerator,
                                //   ConvexPolyhedronConvexPolyhedronManifoldGenerator,
                                //   DefaultContactDispatcher,
                                  PlaneBallManifoldGenerator,
                                  PlaneConvexPolyhedronManifoldGenerator};

#[doc(inline)]
pub use self::proximity_detector::{BallBallProximityDetector,
//                                    CompositeShapeShapeProximityDetector,
//                                    DefaultProximityDispatcher, PlaneSupportMapProximityDetector,
                                   ProximityAlgorithm, ProximityDetector, ProximityDispatcher,
//                                    ShapeCompositeShapeProximityDetector,
                                   SupportMapPlaneProximityDetector,
//                                    SupportMapSupportMapProximityDetector
};

#[doc(hidden)]
pub mod contact_generator;
#[doc(hidden)]
pub mod proximity_detector;
#[doc(hidden)]
pub mod narrow_phase;
mod default_narrow_phase;