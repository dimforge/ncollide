use crate::bounding_volume::BoundingVolume;
use na::RealField;
use crate::partitioning::{VisitStatus, Visitor};
use std::marker::PhantomData;

/// Spatial partitioning data structure visitor visiting interferences with a given bounding volume.
pub struct BoundingVolumeInterferencesVisitor<'a, N: 'a, T: 'a, BV: 'a, Visitor: FnMut(&T) -> VisitStatus> {
    /// The bounding volume used for interference tests.
    pub bv: &'a BV,
    /// Visitor function.
    pub visitor: Visitor,
    _data: PhantomData<(N, &'a T)>,
}

impl<'a, N, T, BV, Visitor: FnMut(&T) -> VisitStatus> BoundingVolumeInterferencesVisitor<'a, N, T, BV, Visitor>
where
    N: RealField,
    BV: BoundingVolume<N>,
{
    /// Creates a new `BoundingVolumeInterferencesVisitor`.
    #[inline]
    pub fn new(
        bv: &'a BV,
        visitor: Visitor,
    ) -> Self
    {
        Self {
            bv,
            visitor,
            _data: PhantomData,
        }
    }
}

impl<'a, N, T, BV, VisitorFn: FnMut(&T) -> VisitStatus> Visitor<T, BV> for BoundingVolumeInterferencesVisitor<'a, N, T, BV, VisitorFn>
where
    N: RealField,
    BV: BoundingVolume<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.intersects(self.bv) {
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
