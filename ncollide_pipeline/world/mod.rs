//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::{CollisionQueryType, CollisionObject, CollisionShape, CollisionShapeHandle};
pub use self::collision_groups::{CollisionGroups, CollisionGroupsPairFilter};
pub use self::collision_world::{BroadPhaseObject, CollisionWorld};

use na::{Pnt3, Iso3, Pnt2, Iso2};

mod collision_object;
mod collision_groups;
mod collision_world;


/// A 3D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<Pnt3<N>, Iso3<N>, T>;
/// A 2D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<Pnt2<N>, Iso2<N>, T>;
