use std::borrow;
use nalgebra::traits::vector::Vec;
use bounding_volume::bounding_volume::BoundingVolume;
use ray::ray::{Ray, RayCast};

/// Visitor of Bounding Volume Trees.
pub trait BVTVisitor<B, BV> {
    /// Visits an internal node. Returns `true` if the internal node children have to be visited
    /// too.
    fn visit_internal(&mut self, &BV) -> bool;

    /// Visits a leaf.
    fn visit_leaf(&mut self, &B, &BV);

    /// Visits an internal node. Returns `true` if the internal node children have to be visited
    /// too.
    #[inline]
    fn visit_internal_mut(&mut self, bv: &mut BV) -> bool {
        self.visit_internal(bv)
    }

    /// Visits a leaf.
    #[inline]
    fn visit_leaf_mut(&mut self, b: &mut B, bv: &mut BV) {
        self.visit_leaf(b, bv)
    }
}

/// Bounding Volume Tree visitor collecting interferences with a given ray.
pub struct RayInterferencesCollector<'self, V, B> {
    priv ray:       &'self Ray<V>,
    priv collector: &'self mut ~[B]
}

impl<'self, V, B> RayInterferencesCollector<'self, V, B> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(ray:    &'self Ray<V>,
               buffer: &'self mut ~[B])
               -> RayInterferencesCollector<'self, V, B> {
        RayInterferencesCollector {
            ray:       ray,
            collector: buffer
        }
    }
}

impl<'self,
     N,
     V:  Vec<N>,
     B:  Clone,
     BV: RayCast<N, V>>
BVTVisitor<B, BV> for RayInterferencesCollector<'self, V, B> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects_ray(self.ray)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.intersects_ray(self.ray) {
            self.collector.push(b.clone())
        }
    }
}

/// Bounding Volume Tree visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeInterferencesCollector<'self, B, BV> {
    priv bv:        &'self BV,
    priv collector: &'self mut ~[B]
}

impl<'self, B, BV> BoundingVolumeInterferencesCollector<'self, B, BV> {
    /// Creates a new `BoundingVolumeInterferencesCollector`.
    #[inline]
    pub fn new(bv:     &'self BV,
               buffer: &'self mut ~[B])
               -> BoundingVolumeInterferencesCollector<'self, B, BV> {
        BoundingVolumeInterferencesCollector {
            bv:        bv,
            collector: buffer
        }
    }
}

impl<'self,
     B:  Clone,
     BV: BoundingVolume>
BVTVisitor<B, BV> for BoundingVolumeInterferencesCollector<'self, B, BV> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects(self.bv)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if !borrow::ref_eq(self.bv, bv) && bv.intersects(self.bv) {
            self.collector.push(b.clone())
        }
    }
}
