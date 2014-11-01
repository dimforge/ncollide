//! Collision detection algorithms and structure for the Narrow Phase.
pub use narrow::collision_detector::{CollisionDetector, Contact};
pub use narrow::empty::Empty;
pub use narrow::ball_ball::BallBall;
pub use narrow::plane_implicit::{PlaneImplicit, ImplicitPlane};
pub use narrow::implicit_implicit::ImplicitImplicit;
pub use narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use narrow::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use narrow::geom_geom ::{DynamicCollisionDetector, ShapeShapeCollisionDetector,
                             ShapeShapeDispatcher, CollisionDetectorFactory};
pub use narrow::concave_geom_geom::{ConcaveShapeShape, ShapeConcaveShape,
                                    ConcaveShapeShapeFactory, ShapeConcaveShapeFactory};
pub use narrow::bezier_surface_ball::{BallBezierSurface, BezierSurfaceBall};

/// Computes the closest points between two balls.
pub use narrow::ball_ball::closest_points as closest_points_ball_ball;

/// Functions to compute the time of impact between two geometries.
pub mod toi {
    pub use narrow::ball_ball::toi                    as ball_ball;
    pub use narrow::plane_implicit::toi               as plane_implicit;
    pub use narrow::implicit_implicit::toi            as implicit_implicit;
    pub use narrow::implicit_implicit::toi_and_normal as implicit_implicit_and_normal;
}

/// Functions to compute one contact point between two geometries.
pub mod collide {
    pub use narrow::ball_ball::collide         as ball_ball;
    pub use narrow::plane_implicit::collide    as plane_implicit;
    pub use narrow::implicit_implicit::collide as implicit_implicit;
}

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

// FIXME: move this module somewhere else!
/// Algorithms needed for distance and penetration depth computation.
pub mod algorithm
{
    pub mod simplex;
    pub mod johnson_simplex;
    pub mod gjk;
    pub mod minkowski_sampling;
}

// FIXME: move those modules somewhere else!
pub mod surface_selector;
pub mod surface_subdivision_tree;
