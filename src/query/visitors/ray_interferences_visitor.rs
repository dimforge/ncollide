use crate::math::Isometry;
use na::RealField;
use crate::partitioning::{VisitStatus, Visitor};
use crate::query::{Ray, RayCast};
use std::marker::PhantomData;

/// Bounding Volume Tree visitor visiting interferences with a given ray.
pub struct RayInterferencesVisitor<'a, N: RealField, T, Visitor: FnMut(&T) -> VisitStatus> {
    /// Ray to be tested.
    pub ray: &'a Ray<N>,
    /// Visitor function.
    pub visitor: Visitor,
    _data: PhantomData<&'a T>,
}

impl<'a, N: RealField, T, Visitor: FnMut(&T) -> VisitStatus> RayInterferencesVisitor<'a, N, T, Visitor> {
    /// Creates a new `RayInterferencesVisitor`.
    #[inline]
    pub fn new(ray: &'a Ray<N>, visitor: Visitor) -> Self {
        Self {
            ray,
            visitor,
            _data: PhantomData,
        }
    }
}

impl<'a, N, T, VisitorFn: FnMut(&T) -> VisitStatus, BV> Visitor<T, BV> for RayInterferencesVisitor<'a, N, T, VisitorFn>
where
    N: RealField,
    BV: RayCast<N>,
{
    #[inline]
    fn visit(&mut self, bv: &BV, t: Option<&T>) -> VisitStatus {
        if bv.intersects_ray(&Isometry::identity(), self.ray) {
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
