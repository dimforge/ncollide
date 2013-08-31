// types and traits
pub use narrow::private::collision_detector::CollisionDetector;
pub use narrow::private::empty::Empty;
pub use narrow::private::ball_ball::BallBall;
pub use narrow::private::geom_geom::GeomGeom;
pub use narrow::private::plane_implicit::{PlaneImplicit, ImplicitPlane};
pub use narrow::private::implicit_implicit::ImplicitImplicit;
pub use narrow::private::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use narrow::private::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use narrow::private::compound_any::{CompoundAABBAny, AnyCompoundAABB};
pub use narrow::private::compound_compound::CompoundAABBCompoundAABB;

// functions
/// Functions to compule the time of impact between two geometries.
pub mod toi {
    pub use ball_ball         = narrow::private::ball_ball::toi;
    pub use plane_implicit    = narrow::private::plane_implicit::toi;
    pub use implicit_implicit = narrow::private::implicit_implicit::toi;
    pub use geom_geom         = narrow::private::geom_geom::toi;
}

/// Functions to compute one contact point between two geometries.
pub mod collide {
    pub use ball_ball         = narrow::private::ball_ball::collide;
    pub use plane_implicit    = narrow::private::plane_implicit::collide;
    pub use implicit_implicit = narrow::private::implicit_implicit::collide;
}

/// Functions to compute the closest points between two geometries.
pub mod closest_points {
    pub use ball_ball         = narrow::private::ball_ball::closest_points;
}

// modules
mod private { // FIXME: this is only to do the compiler's work: ensure invisibility of submodules.
    #[path = "../collision_detector.rs"]
    mod collision_detector;

    #[path = "../empty.rs"]
    mod empty;

    #[path = "../geom_geom.rs"]
    mod geom_geom;

    #[path = "../ball_ball.rs"]
    mod ball_ball;

    #[path = "../plane_implicit.rs"]
    mod plane_implicit;

    #[path = "../implicit_implicit.rs"]
    mod implicit_implicit;

    #[path = "../incremental_contact_manifold_generator.rs"]
    mod incremental_contact_manifold_generator;

    #[path = "../one_shot_contact_manifold_generator.rs"]
    mod one_shot_contact_manifold_generator;

    #[path = "../compound_any.rs"]
    mod compound_any;

    #[path = "../compound_compound.rs"]
    mod compound_compound;

}

pub mod algorithm
{
    pub mod simplex;
    pub mod johnson_simplex;
    pub mod brute_force_simplex;
    pub mod gjk;
    pub mod minkowski_sampling;
}
