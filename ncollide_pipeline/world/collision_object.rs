use math::{Scalar, Point, Vector};
use geometry::shape::ShapeHandle;
use world::CollisionGroups;

/// The kind of query a CollisionObject may be involved on.
///
/// The following queries are executed for a given pair of `GeometricQueryType` associated with two
/// collision objects:
///
/// * Contacts + Contacts = exact contact point coputation.
/// * Contacts + Proximity = proximity test only.
/// * Proximity + Proximity = proximity test only.
#[derive(Debug, PartialEq, Clone, Copy, RustcEncodable, RustcDecodable)]
pub enum GeometricQueryType<N: Scalar> {
    /// This objects can respond to both contact point computation and proximity queries.
    Contacts(N),
    /// This object can respond to proximity tests only.
    Proximity(N),
    // FIXME: not yet implemented: Distance
}

impl<N: Scalar> GeometricQueryType<N> {
    /// The numerical limit of relevance for this query.
    ///
    /// If two objects are separated by a distance greater than the sum of their respective
    /// `query_limit`, the corresponding query will not by performed. For proximity queries,
    /// non-intersecting object closer than a distance equal to the sum of their `query_limit` will
    /// be reported as `Proximity::WithinMargin`.
    #[inline]
    pub fn query_limit(&self) -> N {
        match *self {
            GeometricQueryType::Contacts(ref val)  => *val,
            GeometricQueryType::Proximity(ref val) => *val
        }
    }

    /// Returns `true` if this is a contacts query type.
    #[inline]
    pub fn is_contacts_query(&self) -> bool {
        if let GeometricQueryType::Contacts(_) = *self {
            true
        }
        else {
            false
        }
    }

    /// Returns `true` if this is a proximity query type.
    #[inline]
    pub fn is_proximity_query(&self) -> bool {
        if let GeometricQueryType::Proximity(_) = *self {
            true
        }
        else {
            false
        }
    }
}

// FIXME: really keep all the fields public?
/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<P: Point, M, T> {
    /// The collsion object unique identifier.
    pub uid: usize,
    /// The collision object position.
    pub position: M,
    /// The collision object shape.
    pub shape: ShapeHandle<P, M>,
    /// The collision groups of the collision object.
    pub collision_groups: CollisionGroups,
    /// The kind of queries this collision object is expected to .
    pub query_type: GeometricQueryType<<P::Vect as Vector>::Scalar>,
    /// The user-defined data associated to this object.
    pub data: T,
    #[doc(hidden)]
    pub timestamp: usize
}

impl<P: Point, M, T> CollisionObject<P, M, T> {
    /// Creates a new collision object.
    pub fn new(uid:        usize,
               position:   M,
               shape:      ShapeHandle<P, M>,
               groups:     CollisionGroups,
               query_type: GeometricQueryType<<P::Vect as Vector>::Scalar>,
               data:       T)
               -> CollisionObject<P, M, T> {
        CollisionObject {
            uid:              uid,
            position:         position,
            shape:            shape,
            collision_groups: groups,
            data:             data,
            query_type:       query_type,
            timestamp:        0
        }
    }
}
