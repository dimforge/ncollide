use bounding_volume::BoundingVolume;
use ray::{Ray, RayCast};

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
pub struct RayInterferencesCollector<'a, B> {
    ray:       &'a Ray,
    collector: &'a mut Vec<B>
}

impl<'a, B> RayInterferencesCollector<'a, B> {
    /// Creates a new `RayInterferencesCollector`.
    #[inline]
    pub fn new(ray:    &'a Ray,
               buffer: &'a mut Vec<B>)
               -> RayInterferencesCollector<'a, B> {
        RayInterferencesCollector {
            ray:       ray,
            collector: buffer
        }
    }
}

impl<'a, B: Clone, BV: RayCast> BVTVisitor<B, BV> for RayInterferencesCollector<'a, B> {
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
pub struct BoundingVolumeInterferencesCollector<'a, B, BV> {
    bv:        &'a BV,
    collector: &'a mut Vec<B>
}

impl<'a, B, BV> BoundingVolumeInterferencesCollector<'a, B, BV> {
    /// Creates a new `BoundingVolumeInterferencesCollector`.
    #[inline]
    pub fn new(bv: &'a BV, buffer: &'a mut Vec<B>) -> BoundingVolumeInterferencesCollector<'a, B, BV> {
        BoundingVolumeInterferencesCollector {
            bv:        bv,
            collector: buffer
        }
    }
}

impl<'a, B: Clone, BV: BoundingVolume> BVTVisitor<B, BV> for BoundingVolumeInterferencesCollector<'a, B, BV> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.intersects(self.bv)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if (self.bv as *const BV) != (bv as *const BV) && bv.intersects(self.bv) {
            self.collector.push(b.clone())
        }
    }
}
