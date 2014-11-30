use std::sync::Arc;
use shape::Shape;
use world::CollisionGroups;

/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<N, P, V, M> {
    /// The position of the collision object.
    pub position: M,
    /// The shape of the collision object.
    pub shape: Arc<Box<Shape<N, P, V, M> + Send + Sync>>,
    /// The collision groups of the collision object.
    pub collision_groups: CollisionGroups,

    timestamp: uint
}

impl<N, P, V, M> CollisionObject<N, P, V, M> {
    /// Creates a new collision object.
    pub fn new<S>(position: M, shape: S, groups: CollisionGroups) -> CollisionObject<N, P, V, M>
        where S: Shape<N, P, V, M> + Send + Sync {
        let shape = Arc::new(box shape as Box<Shape<N, P, V, M> + Send + Sync>);
        CollisionObject::new_shared(position, shape, groups)
    }

    /// Creates a new collisio object using a shared shape.
    pub fn new_shared(position: M,
                      shape:    Arc<Box<Shape<N, P, V, M> + Send + Sync>>,
                      groups:   CollisionGroups)
                      -> CollisionObject<N, P, V, M> {
        CollisionObject {
            position:         position,
            shape:            shape,
            collision_groups: groups,
            timestamp:        0
        }
    }

    #[doc(hidden)]
    pub fn set_timestamp(&mut self, timestamp: uint) {
        self.timestamp = timestamp
    }

    #[doc(hidden)]
    pub fn timestamp(&self) -> uint {
        self.timestamp
    }
}
