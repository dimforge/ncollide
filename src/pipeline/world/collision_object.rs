use std::ops::{Index, IndexMut};
use slab::{Iter, Slab};

use alga::general::Real;

use shape::ShapeHandle;
use query::ContactPrediction;
use pipeline::broad_phase::ProxyHandle;
use math::Isometry;

/// The kind of query a CollisionObject may be involved on.
///
/// The following queries are executed for a given pair of `GeometricQueryType` associated with two
/// collision objects:
///
/// * Contacts + Contacts = exact contact point coputation.
/// * Contacts + Proximity = proximity test only.
/// * Proximity + Proximity = proximity test only.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum GeometricQueryType<N: Real> {
    /// This objects can respond to both contact point computation and proximity queries.
    Contacts(N, N),
    /// This object can respond to proximity tests only.
    Proximity(N),
    // FIXME: not yet implemented: Distance
}

impl<N: Real> GeometricQueryType<N> {
    /// The numerical distance limit of relevance for this query.
    ///
    /// If two objects are separated by a distance greater than the sum of their respective
    /// `query_limit`, the corresponding query will not by performed. For proximity queries,
    /// non-intersecting object closer than a distance equal to the sum of their `query_limit` will
    /// be reported as `Proximity::WithinMargin`.
    #[inline]
    pub fn query_limit(&self) -> N {
        match *self {
            GeometricQueryType::Contacts(ref val, _) => *val,
            GeometricQueryType::Proximity(ref val) => *val,
        }
    }

    /// Given two contact query types, returns the corresponding contact prediction parameters.
    ///
    /// Returns `None` if any of `self` or `other` is not a `GeometricQueryType::Contacts`.
    pub fn contact_queries_to_prediction(self, other: Self) -> Option<ContactPrediction<N>> {
        match (self, other) {
            (
                GeometricQueryType::Contacts(linear1, angular1),
                GeometricQueryType::Contacts(linear2, angular2),
            ) => Some(ContactPrediction::new(
                linear1 + linear2,
                angular1,
                angular2,
            )),
            _ => None,
        }
    }

    /// Returns `true` if this is a contacts query type.
    #[inline]
    pub fn is_contacts_query(&self) -> bool {
        if let GeometricQueryType::Contacts(..) = *self {
            true
        } else {
            false
        }
    }

    /// Returns `true` if this is a proximity query type.
    #[inline]
    pub fn is_proximity_query(&self) -> bool {
        if let GeometricQueryType::Proximity(_) = *self {
            true
        } else {
            false
        }
    }
}

/// A stand-alone object that has a position and a shape.
pub struct CollisionObject<N: Real, T> {
    handle: CollisionObjectHandle,
    proxy_handle: ProxyHandle,
    position: Isometry<N>,
    shape: ShapeHandle<N>,
    collision_group: u32,
    query_type: GeometricQueryType<N>,
    data: T,
    // XXX: could this be replaced by an enum (or bitfield)
    // indicating what has been modified?
    pub(crate) timestamp: usize,
}

impl<N: Real, T> CollisionObject<N, T> {
    /// Creates a new collision object.
    pub fn new(
        handle: CollisionObjectHandle,
        proxy_handle: ProxyHandle,
        position: Isometry<N>,
        shape: ShapeHandle<N>,
        collision_group: u32,
        query_type: GeometricQueryType<N>,
        data: T,
    ) -> CollisionObject<N, T> {
        CollisionObject {
            handle: handle,
            proxy_handle: proxy_handle,
            position: position,
            shape: shape,
            collision_group,
            data: data,
            query_type: query_type,
            timestamp: 0,
        }
    }

    /// The collision object unique handle.
    #[inline]
    pub fn handle(&self) -> CollisionObjectHandle {
        self.handle
    }

    #[inline]
    pub(crate) fn set_handle(&mut self, handle: CollisionObjectHandle) {
        self.handle = handle
    }

    /// The collision object's broad phase proxy unique identifier.
    #[inline]
    pub fn proxy_handle(&self) -> ProxyHandle {
        self.proxy_handle
    }

    /// Sets the collision object's broad phase proxy unique identifier.
    #[inline]
    pub(crate) fn set_proxy_handle(&mut self, handle: ProxyHandle) {
        self.proxy_handle = handle
    }

    /// The collision object position.
    #[inline]
    pub fn position(&self) -> &Isometry<N> {
        &self.position
    }

    /// Sets the position of the collision object.
    #[inline]
    pub fn set_position(&mut self, pos: Isometry<N>) {
        self.position = pos
    }

    /// The collision object shape.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<N> {
        &self.shape
    }

    #[inline]
    pub(crate) fn set_shape(&mut self, shape: ShapeHandle<N>) {
        self.shape = shape
    }

    /// The collision groups of the collision object.
    #[inline]
    pub fn collision_group(&self) -> u32 {
        self.collision_group
    }

    #[inline]
    pub(crate) fn set_collision_group(&mut self, group: u32) {
        self.collision_group = group
    }

    /// The kind of queries this collision object is expected to .
    #[inline]
    pub fn query_type(&self) -> GeometricQueryType<N> {
        self.query_type
    }

    /// Sets the `GeometricQueryType` of the collision object.
    #[inline]
    pub fn set_query_type(&mut self, query_type: GeometricQueryType<N>) {
        self.query_type = query_type;
    }

    /// Reference to the user-defined data associated to this object.
    #[inline]
    pub fn data(&self) -> &T {
        &self.data
    }

    /// Mutable reference to the user-defined data associated to this object.
    #[inline]
    pub fn data_mut(&mut self) -> &mut T {
        &mut self.data
    }
}

/// The unique identifier of a collision object.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct CollisionObjectHandle(pub usize);

impl CollisionObjectHandle {
    #[inline]
    pub(crate) fn invalid() -> Self {
        CollisionObjectHandle(usize::max_value())
    }

    /// The unique identifier corresponding to this handle.
    #[inline]
    pub fn uid(&self) -> usize {
        self.0
    }
}

/// A set of collision objects that can be indexed by collision object handles.
pub struct CollisionObjectSlab<N: Real, T> {
    objects: Slab<CollisionObject<N, T>>,
}

impl<N: Real, T> CollisionObjectSlab<N, T> {
    /// Creates a new empty collecton of collision objects.
    pub fn new() -> CollisionObjectSlab<N, T> {
        CollisionObjectSlab {
            objects: Slab::new(),
        }
    }

    /// Inserts a new collision object into this collection and returns the corresponding handle.
    #[inline]
    pub fn insert(&mut self, co: CollisionObject<N, T>) -> CollisionObjectHandle {
        CollisionObjectHandle(self.objects.insert(co))
    }

    /// Removes from this collection the collision object identified by the given handle.
    ///
    /// The removed collision object structure is returned.
    #[inline]
    pub fn remove(&mut self, handle: CollisionObjectHandle) -> CollisionObject<N, T> {
        self.objects.remove(handle.0)
    }

    /// If it exists, retrieves a reference to the collision object identified by the given handle.
    #[inline]
    pub fn get(&self, handle: CollisionObjectHandle) -> Option<&CollisionObject<N, T>> {
        self.objects.get(handle.0)
    }

    /// If it exists, retrieves a mutable reference to the collision object identified by the given handle.
    #[inline]
    pub fn get_mut(&mut self, handle: CollisionObjectHandle) -> Option<&mut CollisionObject<N, T>> {
        self.objects.get_mut(handle.0)
    }

    /// Returns `true` if the specified handle identifies a collision object stored in this collection.
    #[inline]
    pub fn contains(&self, handle: CollisionObjectHandle) -> bool {
        self.objects.contains(handle.0)
    }

    /// Retrieves an iterator yielding references to each collision object.
    #[inline]
    pub fn iter(&self) -> CollisionObjects<N, T> {
        CollisionObjects {
            iter: self.objects.iter(),
        }
    }
}

impl<N: Real, T> Index<CollisionObjectHandle> for CollisionObjectSlab<N, T> {
    type Output = CollisionObject<N, T>;

    #[inline]
    fn index(&self, handle: CollisionObjectHandle) -> &Self::Output {
        &self.objects[handle.0]
    }
}

impl<N: Real, T> IndexMut<CollisionObjectHandle> for CollisionObjectSlab<N, T> {
    #[inline]
    fn index_mut(&mut self, handle: CollisionObjectHandle) -> &mut Self::Output {
        &mut self.objects[handle.0]
    }
}

/// An iterator yielding references to collision objects.
pub struct CollisionObjects<'a, N: 'a + Real, T: 'a> {
    iter: Iter<'a, CollisionObject<N, T>>,
}

impl<'a, N: 'a + Real, T: 'a> Iterator for CollisionObjects<'a, N, T> {
    type Item = &'a CollisionObject<N, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|obj| obj.1)
    }
}
