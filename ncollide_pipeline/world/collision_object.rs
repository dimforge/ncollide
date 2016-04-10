use std::sync::Arc;
use math::{Scalar, Point, Vect};
use entities::inspection::Repr;
use world::CollisionGroups;

/// A dynamically typed geometrical shape of a collision object.
pub type CollisionShape<P, M> = Box<Repr<P, M>>;
/// Handle to a dynamically typed geometrical shape of a collision object.
pub type CollisionShapeHandle<P, M> = Arc<CollisionShape<P, M>>;

/// The kind of query a CollisionObject may be involved on.
///
/// The following queries are executed for a given pair of `CollisionQueryType` associated with two
/// collision objects:
///     * Contacts + Contacts = exact contact point coputation.
///     * Contacts + Proximity = proximity test only.
///     * Proximity + Proximity = proximity test only.
#[derive(Debug, PartialEq, Clone, Copy, RustcEncodable, RustcDecodable)]
pub enum CollisionQueryType<N: Scalar> {
    /// This objects can respond to both contact point computation and proximity queries.
    Contacts(N),
    /// This object can respond to proximity tests only.
    Proximity(N),
    // FIXME: not yet implemented: Distance
}

impl<N: Scalar> CollisionQueryType<N> {
    /// The numerical limit of relevance for this query.
    ///
    /// If two objects are separated by a distance greater than the sum of their respective
    /// `query_limit`, the corresponding query will not by performed. For proximity queries,
    /// non-intersecting object closer than a distance equal to the sum of their `query_limit` will
    /// be reported as `Proximity::WithinMargin`.
    #[inline]
    pub fn query_limit(&self) -> N {
        match *self {
            CollisionQueryType::Contacts(ref val)  => *val,
            CollisionQueryType::Proximity(ref val) => *val
        }
    }
}

// FIXME: really keep all the fields public?
/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<P: Point, M, T> {
    /// The collision object position.
    pub position: M,
    /// The collision object shape.
    pub shape: CollisionShapeHandle<P, M>,
    /// The collision groups of the collision object.
    pub collision_groups: CollisionGroups,
    /// The kind of queries this collision object is expected to .
    pub query_type: CollisionQueryType<<P::Vect as Vect>::Scalar>,
    /// The user-defined data associated to this object.
    pub data: T,
    #[doc(hidden)]
    pub timestamp: usize
}

impl<P: Point, M, T> CollisionObject<P, M, T> {
    /// Creates a new collision object.
    pub fn new(position:   M,
               shape:      CollisionShapeHandle<P, M>,
               groups:     CollisionGroups,
               query_type: CollisionQueryType<<P::Vect as Vect>::Scalar>,
               data:       T)
               -> CollisionObject<P, M, T> {
        CollisionObject {
            position:         position,
            shape:            shape,
            collision_groups: groups,
            data:             data,
            query_type:       query_type,
            timestamp:        0
        }
    }
}
