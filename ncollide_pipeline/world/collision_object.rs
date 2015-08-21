use std::sync::Arc;
use entities::inspection::Repr;
use world::CollisionGroups;

// FIXME: really keep all the fields public?
/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<P, M, T> {
    /// The position of the collision object.
    pub position: M,
    /// The shape of the collision object.
    pub shape: Arc<Box<Repr<P, M>>>,
    /// The collision groups of the collision object.
    pub collision_groups: CollisionGroups,
    /// The data associated to this object.
    pub data: T,
    #[doc(hidden)]
    pub timestamp: usize
}

impl<P, M, T> CollisionObject<P, M, T> {
    /// Creates a new collision object.
    pub fn new(position: M,
               shape:    Arc<Box<Repr<P, M>>>,
               groups:   CollisionGroups,
               data:     T)
               -> CollisionObject<P, M, T> {
        CollisionObject {
            position:         position,
            shape:            shape,
            collision_groups: groups,
            data:             data,
            timestamp:        0
        }
    }
}
