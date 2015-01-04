use bounding_volume::BoundingVolume;

/// Visitor of Bounding Volume Trees.
pub trait BVTVisitor<B, BV> {
    /// Visits an internal node. Returns `true` if the internal node children have to be visited
    /// too.
    fn visit_internal(&mut self, &BV) -> bool;

    /// Visits a leaf.
    fn visit_leaf(&mut self, &B, &BV);
}

/// Bounding Volume Tree visitor collecting interferences with a given bounding volume.
pub struct BoundingVolumeInterferencesCollector<'a, B: 'a, BV: 'a> {
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

impl<'a, N, B: Clone, BV: BoundingVolume<N>> BVTVisitor<B, BV> for BoundingVolumeInterferencesCollector<'a, B, BV> {
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
