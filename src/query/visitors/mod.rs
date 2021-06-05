//! Visitors for performing geometric queries exploiting spatial partitioning data structures.

pub use self::aabb_sets_interferences_visitor::AABBSetsInterferencesVisitor;
pub use self::bounding_volume_interferences_visitor::BoundingVolumeInterferencesVisitor;
pub use self::composite_closest_point_visitor::CompositeClosestPointVisitor;
pub use self::composite_point_containment_test::CompositePointContainmentTest;
pub use self::point_interferences_visitor::PointInterferencesVisitor;
pub use self::ray_interferences_visitor::RayInterferencesVisitor;
pub use self::ray_intersection_cost_fn_visitor::RayIntersectionCostFnVisitor;

mod aabb_sets_interferences_visitor;
mod bounding_volume_interferences_visitor;
mod composite_closest_point_visitor;
mod composite_point_containment_test;
mod point_interferences_visitor;
mod ray_interferences_visitor;
mod ray_intersection_cost_fn_visitor;
