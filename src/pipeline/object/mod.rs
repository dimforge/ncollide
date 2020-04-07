//! Definition of collision objects and some of their properties.

pub use self::collision_groups::{CollisionGroups, CollisionGroupsPairFilter};
pub use self::collision_object::{
    CollisionObject, CollisionObjectRef, CollisionObjectSlabHandle, CollisionObjectUpdateFlags,
};
pub use self::collision_object_set::{
    CollisionObjectHandle, CollisionObjectSet, CollisionObjectSlab, CollisionObjects,
};
pub use self::query_type::GeometricQueryType;

mod collision_groups;
mod collision_object;
mod collision_object_set;
mod query_type;
