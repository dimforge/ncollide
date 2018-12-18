//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::{CollisionObject, CollisionObjectHandle, CollisionObjectSlab,
                                 CollisionObjects, GeometricQueryType};
pub use self::collision_world::{BroadPhaseObject, CollisionWorld, NarrowPhaseObject,
                                InterferencesWithAABB, InterferencesWithPoint,
                                InterferencesWithRay};

mod collision_object;
mod collision_world;