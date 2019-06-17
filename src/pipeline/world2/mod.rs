//! High level API to detect collisions in large, complex scenes.

//pub use self::collision_object::{
//    CollisionObject, CollisionObjectHandle, CollisionObjectSlab, CollisionObjects,
//    GeometricQueryType,
//};
//pub use self::collision_world::{
//    BroadPhaseObject, CollisionWorld, InterferencesWithAABB, InterferencesWithPoint,
//    InterferencesWithRay,
//};
pub use self::collision_object::{CollisionObjectRef, CollisionObjectSet};
pub use self::collision_world::{
    InterferencesWithAABB, InterferencesWithPoint, InterferencesWithRay,
    perform_narrow_phase, perform_broad_phase, perform_all_pipeline,
    interferences_with_ray, interferences_with_point, interferences_with_aabb,
    create_proxies, delete_proxies,
    default_broad_phase, default_narrow_phase, default_interaction_graph};

mod collision_object;
mod collision_world;
