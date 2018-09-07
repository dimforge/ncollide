//! Visitors for performing geometric queries exploiting spatial partitioning data structures.

pub use self::bounding_volume_interferences_collector::BoundingVolumeInterferencesCollector;
pub use self::point_containment_test::PointContainmentTest;
pub use self::point_interferences_collector::PointInterferencesCollector;
pub use self::ray_interferences_collector::RayInterferencesCollector;


mod bounding_volume_interferences_collector;
mod point_containment_test;
mod point_interferences_collector;
mod ray_interferences_collector;