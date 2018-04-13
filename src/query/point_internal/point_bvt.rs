use na::Real;
use partitioning::BVTVisitor;
use query::PointQuery;
use math::{Point, Isometry};

// FIXME: add a point cost fn.

/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
pub struct PointInterferencesCollector<'a, N: 'a + Real, B: 'a> {
    point: &'a Point<N>,
    collector: &'a mut Vec<B>,
}

impl<'a, N: Real, B> PointInterferencesCollector<'a, N, B> {
    /// Creates a new `PointInterferencesCollector`.
    #[inline]
    pub fn new(point: &'a Point<N>, buffer: &'a mut Vec<B>) -> PointInterferencesCollector<'a, N, B> {
        PointInterferencesCollector {
            point: point,
            collector: buffer,
        }
    }
}

impl<'a, N, B, BV> BVTVisitor<B, BV> for PointInterferencesCollector<'a, N, B>
where
    N: Real,
    B: Clone,
    BV: PointQuery<N>,
{
    #[inline]
    fn visit_internal(&mut self, bv: &BV) -> bool {
        bv.contains_point(&Isometry::identity(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &B, bv: &BV) {
        if bv.contains_point(&Isometry::identity(), self.point) {
            self.collector.push(b.clone())
        }
    }
}
