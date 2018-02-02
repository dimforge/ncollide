use std::ops::{Index, IndexMut};
use slab::{Iter, Slab};

use alga::general::Real;

use math::Point;
use geometry::shape::ShapeHandle;
use broad_phase::ProxyHandle;
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
pub enum GeometricQueryType<N: Real> {
    /// This objects can respond to both contact point computation and proximity queries.
    Contacts(N),
    /// This object can respond to proximity tests only.
    Proximity(N),
    // FIXME: not yet implemented: Distance
}

impl<N: Real> GeometricQueryType<N> {
    /// The numerical limit of relevance for this query.
    ///
    /// If two objects are separated by a distance greater than the sum of their respective
    /// `query_limit`, the corresponding query will not by performed. For proximity queries,
    /// non-intersecting object closer than a distance equal to the sum of their `query_limit` will
    /// be reported as `Proximity::WithinMargin`.
    #[inline]
    pub fn query_limit(&self) -> N {
        match *self {
            GeometricQueryType::Contacts(ref val) => *val,
            GeometricQueryType::Proximity(ref val) => *val,
        }
    }

    /// Returns `true` if this is a contacts query type.
    #[inline]
    pub fn is_contacts_query(&self) -> bool {
        if let GeometricQueryType::Contacts(_) = *self {
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
pub struct CollisionObject<P: Point, M, T> {
    handle: CollisionObjectHandle,
    proxy_handle: ProxyHandle,
    position: M,
    shape: ShapeHandle<P, M>,
    collision_groups: CollisionGroups,
    query_type: GeometricQueryType<P::Real>,
    data: T,
    // XXX: could this be replaced by an enum (or bitfield)
    // indicating what has been modified?
    pub(crate) timestamp: usize,
}

impl<P: Point, M, T> CollisionObject<P, M, T> {
    /// Creates a new collision object.
    pub fn new(
        handle: CollisionObjectHandle,
        proxy_handle: ProxyHandle,
        position: M,
        shape: ShapeHandle<P, M>,
        groups: CollisionGroups,
        query_type: GeometricQueryType<P::Real>,
        data: T,
    ) -> CollisionObject<P, M, T> {
        CollisionObject {
            handle: handle,
            proxy_handle: proxy_handle,
            position: position,
            shape: shape,
            collision_groups: groups,
            data: data,
            query_type: query_type,
            timestamp: 0,
        }
    }

    /// The collision object unique identifier.
    #[inline]
    pub fn handle(&self) -> CollisionObjectHandle {
        self.handle
    }

    /// Sets the collision object unique identifier.
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
    pub fn position(&self) -> &M {
        &self.position
    }

    /// Sets the position of the collision object.
    #[inline]
    pub fn set_position(&mut self, pos: M) {
        self.position = pos
    }

    /// The collision object shape.
    #[inline]
    pub fn shape(&self) -> &ShapeHandle<P, M> {
        &self.shape
    }

    /// The collision groups of the collision object.
    #[inline]
    pub fn collision_groups(&self) -> &CollisionGroups {
        &self.collision_groups
    }

    /// The kind of queries this collision object is expected to .
    #[inline]
    pub fn query_type(&self) -> GeometricQueryType<P::Real> {
        self.query_type
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
    pub fn invalid() -> Self {
        CollisionObjectHandle(usize::max_value())
    }

    #[inline]
    pub fn is_invalid(&self) -> bool {
        self.0 == usize::max_value()
    }

    #[inline]
    pub fn uid(&self) -> usize {
        self.0
    }
}

pub struct CollisionObjectSlab<P: Point, M, T> {
    objects: Slab<CollisionObject<P, M, T>>,
}

impl<P: Point, M, T> CollisionObjectSlab<P, M, T> {
    pub fn new() -> CollisionObjectSlab<P, M, T> {
        CollisionObjectSlab {
            objects: Slab::new(),
        }
    }
    #[inline]
    pub fn insert(&mut self, co: CollisionObject<P, M, T>) -> CollisionObjectHandle {
        CollisionObjectHandle(self.objects.insert(co))
    }

    #[inline]
    pub fn remove(&mut self, handle: CollisionObjectHandle) -> CollisionObject<P, M, T> {
        self.objects.remove(handle.0)
    }

    #[inline]
    pub fn get(&self, handle: CollisionObjectHandle) -> Option<&CollisionObject<P, M, T>> {
        self.objects.get(handle.0)
    }

    #[inline]
    pub fn get_mut(
        &mut self,
        handle: CollisionObjectHandle,
    ) -> Option<&mut CollisionObject<P, M, T>> {
        self.objects.get_mut(handle.0)
    }

    #[inline]
    pub fn contains(&self, handle: CollisionObjectHandle) -> bool {
        self.objects.contains(handle.0)
    }

    #[inline]
    pub fn iter(&self) -> CollisionObjects<P, M, T> {
        CollisionObjects {
            iter: self.objects.iter(),
        }
    }
}

impl<P: Point, M, T> Index<CollisionObjectHandle> for CollisionObjectSlab<P, M, T> {
    type Output = CollisionObject<P, M, T>;

    #[inline]
    fn index(&self, handle: CollisionObjectHandle) -> &Self::Output {
        &self.objects[handle.0]
    }
}

impl<P: Point, M, T> IndexMut<CollisionObjectHandle> for CollisionObjectSlab<P, M, T> {
    #[inline]
    fn index_mut(&mut self, handle: CollisionObjectHandle) -> &mut Self::Output {
        &mut self.objects[handle.0]
    }
}

pub struct CollisionObjects<'a, P: 'a + Point, M: 'a, T: 'a> {
    iter: Iter<'a, CollisionObject<P, M, T>>,
}

impl<'a, P: 'a + Point, M: 'a, T: 'a> Iterator for CollisionObjects<'a, P, M, T> {
    type Item = (CollisionObjectHandle, &'a CollisionObject<P, M, T>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.iter
            .next()
            .map(|(id, obj)| (CollisionObjectHandle(id), obj))
    }
}
