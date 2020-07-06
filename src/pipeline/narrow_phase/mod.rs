//! Persistent collision detection algorithms to compute contact points.

#[cfg(feature = "dim3")]
pub use self::contact_generator::TriMeshTriMeshManifoldGenerator;
#[doc(inline)]
pub use self::contact_generator::{
    BallBallManifoldGenerator, BallConvexPolyhedronManifoldGenerator,
    CapsuleCapsuleManifoldGenerator, CapsuleShapeManifoldGenerator,
    CompositeShapeCompositeShapeManifoldGenerator, CompositeShapeShapeManifoldGenerator,
    ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator,
    ConvexPolyhedronConvexPolyhedronManifoldGenerator, DefaultContactDispatcher,
    FlippedContactManifoldGenerator, HeightFieldShapeManifoldGenerator, PlaneBallManifoldGenerator,
    PlaneConvexPolyhedronManifoldGenerator,
};
pub use self::events::{ContactEvent, ContactEvents, EventPool, ProximityEvent, ProximityEvents};
pub use self::interaction_graph::{
    CollisionObjectGraphIndex, Interaction, InteractionGraph, TemporaryInteractionIndex,
};
pub use self::narrow_phase::NarrowPhase;
#[doc(inline)]
pub use self::proximity_detector::{
    BallBallProximityDetector, CompositeShapeShapeProximityDetector, DefaultProximityDispatcher,
    PlaneSupportMapProximityDetector, ProximityAlgorithm, ProximityDetector, ProximityDispatcher,
    SupportMapPlaneProximityDetector, SupportMapSupportMapProximityDetector,
};

#[doc(hidden)]
pub mod contact_generator;
mod events;
mod interaction_graph;
mod narrow_phase;
#[doc(hidden)]
pub mod proximity_detector;
