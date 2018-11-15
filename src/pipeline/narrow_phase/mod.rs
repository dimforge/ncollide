//! Persistent collision detection algorithms to compute contact points.

#[doc(inline)]
pub use self::contact_generator::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator,
    ConvexPolyhedronConvexPolyhedronManifoldGenerator, DefaultContactDispatcher,
    PlaneBallManifoldGenerator, PlaneConvexPolyhedronManifoldGenerator,
};
#[cfg(feature = "dim3")]
pub use self::contact_generator::{TriMeshShapeManifoldGenerator, TriMeshTriMeshManifoldGenerator};
pub use self::default_narrow_phase::DefaultNarrowPhase;
pub use self::narrow_phase::{ContactPairs, NarrowPhase, ProximityPairs};
#[doc(inline)]
pub use self::proximity_detector::{
    BallBallProximityDetector, CompositeShapeShapeProximityDetector, DefaultProximityDispatcher,
    PlaneSupportMapProximityDetector, ProximityAlgorithm, ProximityDetector, ProximityDispatcher,
    SupportMapPlaneProximityDetector, SupportMapSupportMapProximityDetector,
};

#[doc(hidden)]
pub mod contact_generator;
mod default_narrow_phase;
#[doc(hidden)]
pub mod narrow_phase;
#[doc(hidden)]
pub mod proximity_detector;
