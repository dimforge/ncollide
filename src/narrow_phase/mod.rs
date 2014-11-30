//! Collision detection algorithms and structure for the Narrow Phase.
#[doc(inline)]
pub use self::collision_detector::CollisionDetector;
pub use self::empty::Empty;
pub use self::ball_ball::BallBall;
pub use self::plane_support_map::{PlaneSupportMap, SupportMapPlane};
pub use self::support_map_support_map::SupportMapSupportMap;
pub use self::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
pub use self::one_shot_contact_manifold_generator::OneShotContactManifoldGenerator;
#[doc(inline)]
pub use self::shape_shape::{DynamicCollisionDetector, ShapeShapeCollisionDetector,
                            ShapeShapeDispatcher, CollisionDetectorFactory};
pub use self::concave_shape_shape::{ConcaveShapeShape, ShapeConcaveShape,
                                    ConcaveShapeShapeFactory, ShapeConcaveShapeFactory};
pub use self::bezier_surface_ball::{BallBezierSurface, BezierSurfaceBall};
#[doc(inline)]
pub use self::contact_signal::{ContactSignal, ContactSignalHandler};

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

#[doc(hidden)]
pub mod collision_detector;
#[doc(hidden)]
pub mod contact_signal;
mod empty;
mod ball_ball;
mod plane_support_map;
mod support_map_support_map;
mod incremental_contact_manifold_generator;
mod one_shot_contact_manifold_generator;
mod concave_shape_shape;
#[doc(hidden)]
pub mod shape_shape;
mod bezier_surface_ball;

// FIXME: move those modules somewhere else!
pub mod surface_selector;
pub mod surface_subdivision_tree;

/// 2D Shape against Shape collision detection dispatcher.
pub type ShapeShapeDispatcher2<N> = ShapeShapeDispatcher<N, Pnt2<N>, Vec2<N>, Iso2<N>>;
/// 2D Plane against Support Map collision detection dispatcher.
pub type PlaneSupportMap2<N, G> = PlaneSupportMap<N, Pnt2<N>, Vec2<N>, Iso2<N>, G>;

/// 3D Shape against Shape collision detection dispatcher.
pub type ShapeShapeDispatcher3<N> = ShapeShapeDispatcher<N, Pnt3<N>, Vec3<N>, Iso3<N>>;
/// 3D Plane against Support Map collision detection dispatcher.
pub type PlaneSupportMap3<N, G> = PlaneSupportMap<N, Pnt3<N>, Vec3<N>, Iso3<N>, G>;
