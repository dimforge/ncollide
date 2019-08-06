//! Persistent collision detection algorithms to compute contact points.

#[doc(inline)]
pub use self::contact_generator::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator,
    ConvexPolyhedronConvexPolyhedronManifoldGenerator, DefaultContactDispatcher,
    PlaneBallManifoldGenerator, PlaneConvexPolyhedronManifoldGenerator, CapsuleShapeManifoldGenerator,
    CapsuleCapsuleManifoldGenerator, HeightFieldShapeManifoldGenerator
};
#[cfg(feature = "dim3")]
pub use self::contact_generator::{TriMeshTriMeshManifoldGenerator};
pub use self::narrow_phase::NarrowPhase;
#[doc(inline)]
pub use self::proximity_detector::{
    BallBallProximityDetector, CompositeShapeShapeProximityDetector, DefaultProximityDispatcher,
    PlaneSupportMapProximityDetector, ProximityAlgorithm, ProximityDetector, ProximityDispatcher,
    SupportMapPlaneProximityDetector, SupportMapSupportMapProximityDetector,
};
pub use self::interaction_graph::{InteractionGraph, CollisionObjectGraphIndex,
                                  TemporaryInteractionIndex, Interaction};
pub use self::events::{ProximityEvents, ContactEvents, ProximityEvent, ContactEvent, EventPool};

#[doc(hidden)]
pub mod contact_generator;
mod narrow_phase;
#[doc(hidden)]
pub mod proximity_detector;
mod interaction_graph;
mod events;