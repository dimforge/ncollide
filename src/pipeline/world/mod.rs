//! High level API to detect collisions in large, complex scenes.

pub use self::collision_groups::{CollisionGroups, CollisionGroupsPairFilter};
pub use self::collision_object::{
    CollisionObject, CollisionObjectHandle, CollisionObjectSlab, CollisionObjects,
    GeometricQueryType,
};
pub use self::collision_world::{
    BroadPhaseObject, CollisionWorld, InterferencesWithAABB, InterferencesWithPoint,
    InterferencesWithRay,
    SweepTestIntersection,
};

mod collision_groups;
mod collision_object;
mod collision_world;
