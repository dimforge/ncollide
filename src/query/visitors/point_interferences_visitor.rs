use crate::math::{Isometry, Point};
use crate::partitioning::{VisitStatus, Visitor};
use crate::query::PointQuery;
use na::RealField;
use std::marker::PhantomData;

// FIXME: add a point cost fn.

/// Spatial partitioning structure visiting nodes that may contain a given point.
pub struct PointInterferencesVisitor<
    'a,
    N: 'a + RealField,
    T: 'a,
    Visitor: FnMut(&T) -> VisitStatus,
> {
    /// Point to be tested.
    pub point: &'a Point<N>,
    /// Visitor function.
    pub visitor: Visitor,
    _data: PhantomData<&'a T>,
}

impl<'a, N: RealField, T, Visitor: FnMut(&T) -> VisitStatus>
    PointInterferencesVisitor<'a, N, T, Visitor>
{
    /// Creates a new `PointInterferencesVisitor`.
    #[inline]
    pub fn new(
        point: &'a Point<N>,
        visitor: Visitor,
    ) -> PointInterferencesVisitor<'a, N, T, Visitor> {
        PointInterferencesVisitor {
            point,
            visitor,
            _data: PhantomData,
        }
    }
}

impl<'a, N, T, VisitorFn: FnMut(&T) -> VisitStatus, BV> Visitor<T, BV>
    for PointInterferencesVisitor<'a, N, T, VisitorFn>
where
    N: RealField,
    BV: PointQuery<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.contains_point(&Isometry::identity(), self.point) {
            if let Some(t) = t {
                (self.visitor)(t)
            } else {
                VisitStatus::Continue
            }
        } else {
            VisitStatus::Stop
        }
    }
}
