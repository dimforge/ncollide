//! Persistant collision detection algorithms to compute contact points.
pub use self::ball_ball_manifold_generator::BallBallManifoldGenerator;
pub use self::ball_convex_polyhedron_manifold_generator::BallConvexPolyhedronManifoldGenerator;
pub use self::multiball_convex_polyhedron_manifold_generator::MultiballConvexPolyhedronManifoldGenerator;
pub use self::composite_shape_composite_shape_manifold_generator::CompositeShapeCompositeShapeManifoldGenerator;
pub use self::composite_shape_shape_manifold_generator::CompositeShapeShapeManifoldGenerator;
#[doc(inline)]
pub use self::contact_manifold_generator::{
    ContactAlgorithm, ContactDispatcher, ContactManifoldGenerator,
};
pub use self::convex_polyhedron_convex_polyhedron_manifold_generator::ConvexPolyhedronConvexPolyhedronManifoldGenerator;
pub use self::default_contact_dispatcher::DefaultContactDispatcher;
pub use self::plane_ball_manifold_generator::PlaneBallManifoldGenerator;
pub use self::plane_convex_polyhedron_manifold_generator::PlaneConvexPolyhedronManifoldGenerator;
#[cfg(feature = "dim3")]
pub use self::trimesh_trimesh_manifold_generator::TriMeshTriMeshManifoldGenerator;
pub use self::heightfield_shape_manifold_generator::HeightFieldShapeManifoldGenerator;
pub use self::capsule_shape_manifold_generator::CapsuleShapeManifoldGenerator;
pub use self::capsule_capsule_manifold_generator::CapsuleCapsuleManifoldGenerator;

// // FIXME: un-hide this and move everything to a folder.
mod ball_ball_manifold_generator;
mod ball_convex_polyhedron_manifold_generator;
mod multiball_convex_polyhedron_manifold_generator;
mod composite_shape_composite_shape_manifold_generator;
mod composite_shape_shape_manifold_generator;
#[doc(hidden)]
pub mod contact_manifold_generator;
mod convex_polyhedron_convex_polyhedron_manifold_generator;
mod default_contact_dispatcher;
mod plane_ball_manifold_generator;
mod plane_convex_polyhedron_manifold_generator;
#[cfg(feature = "dim3")]
mod trimesh_trimesh_manifold_generator;
mod heightfield_shape_manifold_generator;
mod capsule_shape_manifold_generator;
mod capsule_capsule_manifold_generator;