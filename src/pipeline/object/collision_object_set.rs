use alga::general::RealField;

use slab::{Iter, Slab};
use std::ops::{Index, IndexMut};
use std::hash::Hash;
use crate::pipeline::object::{CollisionObject, CollisionObjectSlabHandle, CollisionObjectRef};

pub trait CollisionObjectHandle: Copy + Hash + PartialEq + Eq + 'static + Send + Sync {
}

impl<T: Copy + Hash + PartialEq + Eq + 'static + Send + Sync> CollisionObjectHandle for T {}

pub trait CollisionObjectSet<N: RealField> {
    type CollisionObject: CollisionObjectRef<N>;
    type CollisionObjectHandle: CollisionObjectHandle;

    fn collision_object(&self, handle: Self::CollisionObjectHandle) -> Option<&Self::CollisionObject>;
    fn foreach(&self, f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject));
}

impl<N: RealField, T> CollisionObjectSet<N> for CollisionObjectSlab<N, T> {
    type CollisionObject = CollisionObject<N, T>;
    type CollisionObjectHandle = CollisionObjectSlabHandle;

    fn collision_object(&self, handle: Self::CollisionObjectHandle) -> Option<&Self::CollisionObject> {
        self.get(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        for co in self.objects.iter() {
            f(CollisionObjectSlabHandle(co.0), co.1)
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
    pub fn insert(&mut self, co: CollisionObject<N, T>) -> CollisionObjectSlabHandle {
        CollisionObjectSlabHandle(self.objects.insert(co))
    }

    /// Removes from this collection the collision object identified by the given handle.
    ///
    /// The removed collision object structure is returned.
    #[inline]
    pub fn remove(&mut self, handle: CollisionObjectSlabHandle) -> CollisionObject<N, T> {
        self.objects.remove(handle.0)
    }

    /// If it exists, retrieves a reference to the collision object identified by the given handle.
    #[inline]
    pub fn get(&self, handle: CollisionObjectSlabHandle) -> Option<&CollisionObject<N, T>> {
        self.objects.get(handle.0)
    }

    /// If it exists, retrieves a mutable reference to the collision object identified by the given handle.
    #[inline]
    pub fn get_mut(&mut self, handle: CollisionObjectSlabHandle) -> Option<&mut CollisionObject<N, T>> {
        self.objects.get_mut(handle.0)
    }

    /// If they exists, retrieves a mutable reference to the two collision object identified by the given handles.
    ///
    /// Panics if both handles are equal.
    #[inline]
    pub fn get_pair_mut(&mut self,
                        handle1: CollisionObjectSlabHandle,
                        handle2: CollisionObjectSlabHandle)
                        -> (Option<&mut CollisionObject<N, T>>, Option<&mut CollisionObject<N, T>>) {
        assert_ne!(handle1, handle2, "The two handles must not be the same.");
        let a = self.objects.get_mut(handle1.0).map(|o| o as *mut _);
        (a.map(|a| unsafe { std::mem::transmute(a) }), self.objects.get_mut(handle2.0))
    }

    /// Returns `true` if the specified handle identifies a collision object stored in this collection.
    #[inline]
    pub fn contains(&self, handle: CollisionObjectSlabHandle) -> bool {
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

impl<N: RealField, T> Index<CollisionObjectSlabHandle> for CollisionObjectSlab<N, T> {
    type Output = CollisionObject<N, T>;

    #[inline]
    fn index(&self, handle: CollisionObjectSlabHandle) -> &Self::Output {
        &self.objects[handle.0]
    }
}

impl<N: RealField, T> IndexMut<CollisionObjectSlabHandle> for CollisionObjectSlab<N, T> {
    #[inline]
    fn index_mut(&mut self, handle: CollisionObjectSlabHandle) -> &mut Self::Output {
        &mut self.objects[handle.0]
    }
}

/// An iterator yielding references to collision objects.
pub struct CollisionObjects<'a, N: 'a + RealField, T: 'a> {
    iter: Iter<'a, CollisionObject<N, T>>,
}

impl<'a, N: 'a + RealField, T: 'a> Iterator for CollisionObjects<'a, N, T> {
    type Item = (CollisionObjectSlabHandle, &'a CollisionObject<N, T>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.iter.next().map(|obj| ((CollisionObjectSlabHandle(obj.0), obj.1)))
    }
}