//! Persistant collision detection algorithms to compute contact points.
pub use self::ball_ball_manifold_generator::BallBallManifoldGenerator;
pub use self::ball_convex_polyhedron_manifold_generator::BallConvexPolyhedronManifoldGenerator;
pub use self::composite_shape_composite_shape_manifold_generator::CompositeShapeCompositeShapeManifoldGenerator;
pub use self::composite_shape_shape_manifold_generator::CompositeShapeShapeManifoldGenerator;
#[doc(inline)]
pub use self::contact_manifold_generator::{ContactAlgorithm, ContactDispatcher,
                                           ContactManifoldGenerator};
pub use self::convex_polyhedron_convex_polyhedron_manifold_generator::ConvexPolyhedronConvexPolyhedronManifoldGenerator;
pub use self::default_contact_dispatcher::DefaultContactDispatcher;
pub use self::plane_ball_manifold_generator::PlaneBallManifoldGenerator;
pub use self::plane_convex_polyhedron_manifold_generator::PlaneConvexPolyhedronManifoldGenerator;
//#[cfg(feature = "dim3")]
//pub use self::simplicial_complex_simplicial_complex_manifold_generator::SimplicialComplexSimplicialComplexManifoldGenerator;

// // FIXME: un-hide this and move everything to a folder.
#[doc(hidden)]
pub mod contact_manifold_generator;
mod default_contact_dispatcher;
mod ball_ball_manifold_generator;
mod ball_convex_polyhedron_manifold_generator;
mod convex_polyhedron_convex_polyhedron_manifold_generator;
mod plane_ball_manifold_generator;
mod plane_convex_polyhedron_manifold_generator;
mod composite_shape_shape_manifold_generator;
mod composite_shape_composite_shape_manifold_generator;
//#[cfg(feature = "dim3")]
//mod simplicial_complex_simplicial_complex_manifold_generator;
