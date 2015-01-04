//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::CollisionObject;
pub use self::collision_groups::CollisionGroups;
pub use self::collision_objects_dispatcher::CollisionObjectsDispatcher;
pub use self::collision_world::{BroadPhaseObject, CollisionWorld};

use na::{Pnt3, Vec3, Iso3, Pnt2, Vec2, Iso2};

mod collision_object;
mod collision_groups;
mod collision_objects_dispatcher;
mod collision_world;


/// A 3D collision world associating collision objects to user-defined data of type `T`.
type CollisionWorld3<N, T> = CollisionWorld<N, Pnt3<N>, Vec3<N>, Iso3<N>, T>;
/// A 2D collision world associating collision objects to user-defined data of type `T`.
type CollisionWorld2<N, T> = CollisionWorld<N, Pnt2<N>, Vec2<N>, Iso2<N>, T>;
