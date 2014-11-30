//! High level API to detect collisions in large, complex scenes.

pub use self::collision_object::CollisionObject;
pub use self::collision_groups::CollisionGroups;
pub use self::collision_objects_dispatcher::{CollisionObjectsDispatcher,
                                             CollisionObjectsProximityFilter,
                                             AbstractCollisionDetector};
pub use self::collision_world::{BroadPhaseObject, CollisionWorld, CollisionObjectRegister};

use na::{Pnt3, Vec3, Iso3, Pnt2, Vec2, Iso2};

mod collision_object;
mod collision_groups;
mod collision_objects_dispatcher;
mod collision_world;


/// A 3D collision world contaning objects of type `O`.
type CollisionWorld3<N, O> = CollisionWorld<N, Pnt3<N>, Vec3<N>, Iso3<N>, O>;
/// A 2D collision world contaning objects of type `O`.
type CollisionWorld2<N, O> = CollisionWorld<N, Pnt2<N>, Vec2<N>, Iso2<N>, O>;
