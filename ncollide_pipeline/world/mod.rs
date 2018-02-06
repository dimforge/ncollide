//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::{CollisionObject, CollisionObjectHandle, CollisionObjectSlab,
                                 CollisionObjects, GeometricQueryType};
pub use self::collision_groups::{CollisionGroups, CollisionGroupsPairFilter};
pub use self::collision_world::{BroadPhaseObject, CollisionWorld, NarrowPhaseObject,
                                InterferencesWithAABB, InterferencesWithPoint,
                                InterferencesWithRay};

use na::{Isometry2, Isometry3, Point2, Point3};

mod collision_object;
mod collision_groups;
mod collision_world;

/// A 3D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<Point3<N>, Isometry3<N>, T>;
/// A 2D collision world associating collision objects to user-defined data of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<Point2<N>, Isometry2<N>, T>;

/// A 3D collision object with user-defined data of type `T`.
pub type CollisionObject3<N, T> = CollisionObject<Point3<N>, Isometry3<N>, T>;
/// A 2D collision object with user-defined data of type `T`.
pub type CollisionObject2<N, T> = CollisionObject<Point2<N>, Isometry2<N>, T>;
