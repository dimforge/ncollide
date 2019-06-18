use alga::general::RealField;
use crate::math::Isometry;
use crate::pipeline::broad_phase::BroadPhaseProxyHandle;
use crate::pipeline::narrow_phase::CollisionObjectGraphIndex;
use crate::pipeline::object::{CollisionGroups, GeometricQueryType};
use crate::query::ContactPrediction;
use crate::shape::Shape;
use crate::bounding_volume::{self, BoundingVolume, AABB};

use slab::{Iter, Slab};
use std::ops::{Index, IndexMut};
use crate::pipeline::object::{CollisionObject, CollisionObjectHandle, CollisionObjectRef};


pub trait CollisionObjectSet<'a, N: RealField> {
    type CollisionObject: CollisionObjectRef<'a, N, Handle = Self::Handle>;
    type CollisionObjects: Iterator<Item = (Self::Handle, Self::CollisionObject)>;
    type Handle: Copy;

    fn get(&'a self, handle: Self::Handle) -> Option<Self::CollisionObject>;
    fn contains(&self, handle: Self::Handle) -> bool;
    fn iter(&'a self) -> Self::CollisionObjects;
}

impl<'a, N: RealField, T: 'a> CollisionObjectSet<'a, N> for CollisionObjectSlab<N, T> {
    type CollisionObject = &'a CollisionObject<N, T>;
    type CollisionObjects = CollisionObjects<'a, N, T>;
    type Handle = CollisionObjectHandle;

    fn get(&'a self, handle: Self::Handle) -> Option<Self::CollisionObject> {
        self.get(handle)
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.contains(handle)
    }

    fn iter(&'a self) -> Self::CollisionObjects {
        CollisionObjects {
            iter: self.objects.iter()
        }
    }
}

/// A set of collision objects that can be indexed by collision object handles.
pub struct CollisionObjectSlab<N: RealField, T> {
    pub(crate) objects: Slab<CollisionObject<N, T>>,
}

impl<N: RealField, T> CollisionObjectSlab<N, T> {
    /// Creates a new empty collection of collision objects.
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

    /// If they exists, retrieves a mutable reference to the two collision object identified by the given handles.
    ///
    /// Panics if both handles are equal.
    #[inline]
    pub fn get_pair_mut(&mut self,
                        handle1: CollisionObjectHandle,
                        handle2: CollisionObjectHandle)
                        -> (Option<&mut CollisionObject<N, T>>, Option<&mut CollisionObject<N, T>>) {
        assert_ne!(handle1, handle2, "The two handles must not be the same.");
        let a = self.objects.get_mut(handle1.0).map(|o| o as *mut _);
        (a.map(|a| unsafe { std::mem::transmute(a) }), self.objects.get_mut(handle2.0))
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

    /// The number of collision objects on this slab.
    #[inline]
    pub fn len(&self) -> usize {
        self.objects.len()
    }
}

impl<N: RealField, T> Index<CollisionObjectHandle> for CollisionObjectSlab<N, T> {
    type Output = CollisionObject<N, T>;

    #[inline]
    fn index(&self, handle: CollisionObjectHandle) -> &Self::Output {
        &self.objects[handle.0]
    }
}

impl<N: RealField, T> IndexMut<CollisionObjectHandle> for CollisionObjectSlab<N, T> {
    #[inline]
    fn index_mut(&mut self, handle: CollisionObjectHandle) -> &mut Self::Output {
        &mut self.objects[handle.0]
    }
}

/// An iterator yielding references to collision objects.
pub struct CollisionObjects<'a, N: 'a + RealField, T: 'a> {
    iter: Iter<'a, CollisionObject<N, T>>,
}

impl<'a, N: 'a + RealField, T: 'a> Iterator for CollisionObjects<'a, N, T> {
    type Item = (CollisionObjectHandle, &'a CollisionObject<N, T>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|obj| ((CollisionObjectHandle(obj.0), obj.1)))
    }
}