use math::{Isometry, Point};
use na::Real;
use partitioning::{Visitor, VisitStatus};
use query::PointQuery;

// FIXME: add a point cost fn.

/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
pub struct PointInterferencesCollector<'a, N: 'a + Real, T: 'a> {
    pub point: &'a Point<N>,
    pub collector: &'a mut Vec<T>,
}

impl<'a, N: Real, T> PointInterferencesCollector<'a, N, T> {
    /// Creates a new `PointInterferencesCollector`.
    #[inline]
    pub fn new(point: &'a Point<N>, buffer: &'a mut Vec<T>) -> PointInterferencesCollector<'a, N, T> {
        PointInterferencesCollector {
            point: point,
            collector: buffer,
        }
    }
}

impl<'a, N, T, BV> Visitor<T, BV> for PointInterferencesCollector<'a, N, T>
    where
        N: Real,
        T: Clone,
        BV: PointQuery<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.contains_point(&Isometry::identity(), self.point) {
            if let Some(t) = t {
                self.collector.push(t.clone());
            }
            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}
