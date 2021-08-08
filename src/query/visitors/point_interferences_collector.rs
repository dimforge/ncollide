use crate::math::{Isometry, Point};
use crate::partitioning::{VisitStatus, Visitor};
use crate::query::PointQuery;
use na::RealField;

// FIXME: add a point cost fn.

/// Spatial partitioning structure visitor collecting nodes that may contain a given point.
pub struct PointInterferencesCollector<'a, N: 'a + RealField + Copy, T: 'a> {
    /// Point to be tested.
    pub point: &'a Point<N>,
    /// The data contained by the nodes which bounding volume contain `self.point`.
    pub collector: &'a mut Vec<T>,
}

impl<'a, N: RealField + Copy, T> PointInterferencesCollector<'a, N, T> {
    /// Creates a new `PointInterferencesCollector`.
    #[inline]
    pub fn new(
        point: &'a Point<N>,
        buffer: &'a mut Vec<T>,
    ) -> PointInterferencesCollector<'a, N, T> {
        PointInterferencesCollector {
            point: point,
            collector: buffer,
        }
    }
}

impl<'a, N, T, BV> Visitor<T, BV> for PointInterferencesCollector<'a, N, T>
where
    N: RealField + Copy,
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
