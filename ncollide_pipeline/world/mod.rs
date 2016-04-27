//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::{GeometricQueryType, CollisionObject};
pub use self::collision_groups::{CollisionGroups, CollisionGroupsPairFilter};
pub use self::collision_world::{BroadPhaseObject, CollisionWorld};

use na::{Point3, Isometry3, Point2, Isometry2};

mod collision_object;
mod collision_groups;
mod collision_world;


/// A 3D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<Point3<N>, Isometry3<N>, T>;
/// A 2D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<Point2<N>, Isometry2<N>, T>;
