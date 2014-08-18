//! Narrow phases.

// types and traits
pub use narrow::collision_detector::{CollisionDetector, Contact};
pub use narrow::empty::Empty;
pub use narrow::ball_ball::BallBall;
pub use narrow::plane_implicit::{PlaneImplicit, ImplicitPlane};
pub use narrow::implicit_implicit::ImplicitImplicit;
pub use narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use narrow::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use narrow::geom_geom::{DynamicCollisionDetector, GeomGeomCollisionDetector, GeomGeomDispatcher,
                            CollisionDetectorFactory};
pub use narrow::concave_geom_geom::{ConcaveGeomGeom, GeomConcaveGeom, ConcaveGeomGeomFactory,
                                    GeomConcaveGeomFactory};
pub use narrow::bezier_surface_ball::{BallBezierSurface, BezierSurfaceBall};
pub use narrow::bezier_surface_bezier_surface::BezierSurfaceBezierSurface;

// functions
/// Functions to compute the time of impact between two geometries.
pub mod toi {
    pub use ball_ball         = narrow::ball_ball::toi;
    pub use plane_implicit    = narrow::plane_implicit::toi;
    pub use implicit_implicit = narrow::implicit_implicit::toi;
    pub use implicit_implicit_and_normal = narrow::implicit_implicit::toi_and_normal;
}

/// Functions to compute one contact point between two geometries.
pub mod collide {
    pub use ball_ball         = narrow::ball_ball::collide;
    pub use plane_implicit    = narrow::plane_implicit::collide;
    pub use implicit_implicit = narrow::implicit_implicit::collide;
}

/// Functions to compute the closest points between two geometries.
pub mod closest_points {
    pub use ball_ball         = narrow::ball_ball::closest_points;
}

// modules
mod collision_detector;
mod empty;
mod ball_ball;
mod plane_implicit;
mod implicit_implicit;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod concave_geom_geom;
mod geom_geom;
mod bezier_surface_ball;
mod bezier_surface_bezier_surface;

// FIXME: move this module somewhere else!
/// Algorithms needed for distance and penetration depth computation.
pub mod algorithm
{
    pub mod simplex;
    pub mod johnson_simplex;
    pub mod brute_force_simplex;
    pub mod gjk;
    pub mod minkowski_sampling;
}

// FIXME: move those modules somewhere else!
pub mod surface_selector;
pub mod surface_subdivision_tree;
