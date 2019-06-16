use alga::general::RealField;
use crate::math::Isometry;
use crate::pipeline::broad_phase::BroadPhaseProxyHandle;
use crate::pipeline::narrow_phase::CollisionObjectGraphIndex2;
use crate::pipeline::world::{CollisionGroups, GeometricQueryType};
use crate::query::ContactPrediction;
use crate::shape::ShapeHandle;

use slab::{Iter, Slab};
use std::ops::{Index, IndexMut};

bitflags! {
    #[derive(Default)]
    pub struct CollisionObjectUpdateFlags: u8 {
        const POSITION_CHANGED = 0b00000001;
        const SHAPE_CHANGED = 0b000010;
        const COLLISION_GROUPS_CHANGED = 0b000100;
        const QUERY_TYPE_CHANGED = 0b0001000;
    }
}

impl CollisionObjectUpdateFlags {
    pub fn needs_broad_phase_update(&self) -> bool {
        !self.is_empty()
    }

    pub fn needs_narrow_phase_update(&self) -> bool {
        !self.is_empty()
    }

    pub fn needs_bounding_volume_update(&self) -> bool {
        // NOTE: the QUERY_TYPE_CHANGED is included here because the
        // prediction margin may have changed.
        self.intersects(Self::POSITION_CHANGED | Self::SHAPE_CHANGED | Self::QUERY_TYPE_CHANGED)
    }

    pub fn needs_broad_phase_redispatch(&self) -> bool {
        self.intersects(Self::SHAPE_CHANGED | Self::COLLISION_GROUPS_CHANGED | Self::QUERY_TYPE_CHANGED)
    }
}

pub trait CollisionObjectSet<'a, N: RealField> {
    type CollisionObject: CollisionObjectRef<'a, N>;
    type CollisionObjects: Iterator<Item = (Self::Handle, Self::CollisionObject)>;
    type Handle: Copy;

    fn get(&'a self, handle: Self::Handle) -> Option<Self::CollisionObject>;
    fn contains(&self, handle: Self::Handle) -> bool;
    fn iter(&'a self) -> Self::CollisionObjects;
}

pub trait CollisionObjectRef<'a, N: RealField>: Copy {
    fn graph_index(self) -> CollisionObjectGraphIndex2;
    fn proxy_handle(self) -> BroadPhaseProxyHandle;
    fn position(self) -> &'a Isometry<N>;
    fn shape(self) -> &'a ShapeHandle<N>;
    fn collision_groups(self) -> &'a CollisionGroups;
    fn query_type(self) -> GeometricQueryType<N>;
    fn update_flags(self) -> CollisionObjectUpdateFlags;
}

use crate::world::{CollisionObjectSlab, CollisionObject, CollisionObjectHandle};

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

impl<'a, N: RealField, T> CollisionObjectRef<'a, N> for &'a CollisionObject<N, T> {
    fn graph_index(self) -> CollisionObjectGraphIndex2 {
        self.graph_index()
    }

    fn proxy_handle(self) -> BroadPhaseProxyHandle {
        self.proxy_handle()
    }

    fn position(self) -> &'a Isometry<N> {
        self.position()
    }

    fn shape(self) -> &'a ShapeHandle<N> {
        self.shape()
    }

    fn collision_groups(self) -> &'a CollisionGroups {
        self.collision_groups()
    }

    fn query_type(self) -> GeometricQueryType<N> {
        self.query_type()
    }

    fn update_flags(self) -> CollisionObjectUpdateFlags {
        unimplemented!()
    }
}