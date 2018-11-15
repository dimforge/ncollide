//! Visitors for performing geometric queries exploiting spatial partitioning data structures.

pub use self::aabb_sets_interferences_collector::AABBSetsInterferencesCollector;
pub use self::bounding_volume_interferences_collector::BoundingVolumeInterferencesCollector;
pub use self::composite_closest_point_visitor::CompositeClosestPointVisitor;
pub use self::composite_point_containment_test::CompositePointContainmentTest;
pub use self::point_interferences_collector::PointInterferencesCollector;
pub use self::ray_interferences_collector::RayInterferencesCollector;

mod aabb_sets_interferences_collector;
mod bounding_volume_interferences_collector;
mod composite_closest_point_visitor;
mod composite_point_containment_test;
mod point_interferences_collector;
mod ray_interferences_collector;
