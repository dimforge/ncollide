//! Collision detection algorithms and structure for the Narrow Phase.
#[doc(inline)]
pub use narrow_phase::collision_detector::CollisionDetector;
pub use narrow_phase::empty::Empty;
pub use narrow_phase::ball_ball::BallBall;
pub use narrow_phase::plane_support_map::{PlaneSupportMap, SupportMapPlane};
pub use narrow_phase::support_map_support_map::SupportMapSupportMap;
pub use narrow_phase::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use narrow_phase::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
pub use narrow_phase::geom_geom::{DynamicCollisionDetector, ShapeShapeCollisionDetector,
                                  ShapeShapeDispatcher, CollisionDetectorFactory};
pub use narrow_phase::concave_geom_geom::{ConcaveShapeShape, ShapeConcaveShape,
                                    ConcaveShapeShapeFactory, ShapeConcaveShapeFactory};
pub use narrow_phase::bezier_surface_ball::{BallBezierSurface, BezierSurfaceBall};

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3, Mat1, Mat3};

/// Functions to compute the time of impact between two geometries.
pub mod toi {
    pub use narrow_phase::ball_ball::toi                    as ball_ball;
    pub use narrow_phase::plane_support_map::toi               as plane_support_map;
    pub use narrow_phase::support_map_support_map::toi            as support_map_support_map;
    pub use narrow_phase::support_map_support_map::toi_and_normal as support_map_support_map_and_normal;
}

#[doc(hidden)]
pub mod collision_detector;
mod empty;
mod ball_ball;
mod plane_support_map;
mod support_map_support_map;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod concave_geom_geom;
mod geom_geom;
mod bezier_surface_ball;

// FIXME: move those modules somewhere else!
pub mod surface_selector;
pub mod surface_subdivision_tree;

/// 2D Shape against Shape collision detection dispatcher using single precision floats.
pub type ShapeShapeDispatcher2<N> = ShapeShapeDispatcher<N, Pnt2<N>, Vec2<N>, Iso2<N>, Mat1<N>>;
/// 2D Plane against Support Map collision detection dispatcher using single precision floats.
pub type PlaneSupportMap2<N, G> = PlaneSupportMap<N, Pnt2<N>, Vec2<N>, Iso2<N>, G>;

/// 3D Shape against Shape collision detection dispatcher using single precision floats.
pub type ShapeShapeDispatcher3<N> = ShapeShapeDispatcher<N, Pnt3<N>, Vec3<N>, Iso3<N>, Mat3<N>>;
/// 3D Plane against Support Map collision detection dispatcher using single precision floats.
pub type PlaneSupportMap3<N, G> = PlaneSupportMap<N, Pnt3<N>, Vec3<N>, Iso3<N>, G>;
