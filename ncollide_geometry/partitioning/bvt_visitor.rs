use std::marker::PhantomData;
use bounding_volume::BoundingVolume;
use math::Point;

/// Visitor of Bounding Volume Trees.
pub trait BVTVisitor<B, BV> {
    /// Visits an internal node. Returns `true` if the internal node children have to be visited
    /// too.
    fn visit_internal(&mut self, &BV) -> bool;

    /// Visits a leaf.
    fn visit_leaf(&mut self, &B, &BV);
}

/// Bounding Volume Tree visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeInterferencesCollector<'a, P: 'a, B: 'a, BV: 'a> {
    bv:        &'a BV,
    collector: &'a mut Vec<B>,
    _point:    PhantomData<P>
}

impl<'a, P, B, BV> BoundingVolumeInterferencesCollector<'a, P, B, BV>
    where P: Point,
          BV: BoundingVolume<P> {
    /// Creates a new `BoundingVolumeInterferencesCollector`.
    #[inline]
    pub fn new(bv: &'a BV, buffer: &'a mut Vec<B>) -> BoundingVolumeInterferencesCollector<'a, P, B, BV> {
        BoundingVolumeInterferencesCollector {
            bv:        bv,
            collector: buffer,
            _point:    PhantomData
        }
    }
}

impl<'a, P, B, BV> BVTVisitor<B, BV> for BoundingVolumeInterferencesCollector<'a, P, B, BV>
    where P:  Point,
          B:  Clone,
          BV: BoundingVolume<P> {
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
