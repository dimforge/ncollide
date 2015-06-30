use na::Identity;
use entities::partitioning::BVTVisitor;
use point::PointQuery;
use math::Point;

// FIXME: add a point cost fn.

/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
pub struct PointInterferencesCollector<'a, P: 'a, B: 'a> {
    point:     &'a P,
    collector: &'a mut Vec<B>
}

impl<'a, P, B> PointInterferencesCollector<'a, P, B> {
    /// Creates a new `PointInterferencesCollector`.
    #[inline]
    pub fn new(point: &'a P, buffer: &'a mut Vec<B>) -> PointInterferencesCollector<'a, P, B> {
        PointInterferencesCollector {
            point:     point,
            collector: buffer
        }
    }
}

impl<'a, P, B, BV> BVTVisitor<B, BV> for PointInterferencesCollector<'a, P, B>
    where P:  Point,
          B:  Clone,
          BV: PointQuery<P, Identity> {
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.contains_point(&Identity::new(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.contains_point(&Identity::new(), self.point) {
            self.collector.push(b.clone())
        }
    }
}
