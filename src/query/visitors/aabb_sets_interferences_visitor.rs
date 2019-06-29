use crate::bounding_volume::{BoundingVolume, AABB};
use crate::math::{Isometry, Matrix, Vector};
use na::RealField;
use crate::partitioning::{SimultaneousVisitor, VisitStatus};
use std::marker::PhantomData;

/// Spatial partitioning data structure visitor collecting interferences with a given bounding volume.
pub struct AABBSetsInterferencesVisitor<'a, N: RealField, T, Visitor: FnMut(&T, &T) -> VisitStatus> {
    /// The transform from the local-space of the second bounding volumes to the local space of the first.
    pub ls_m2: &'a Isometry<N>,
    /// The absolute value of the rotation matrix representing `ls_m2.rotation`.
    ///
    /// Equals to `ls_m2.rotation.to_rotation.matrix().matrix().abs()`.
    pub ls_m2_abs_rot: &'a Matrix<N>,
    /// A tolerance applied to the interference tests.
    ///
    /// AABB pairs closer than `tolerance` will be reported as intersecting.
    pub tolerence: N,
    /// The data contained by the nodes with bounding volumes intersecting `self.bv`.
    pub visitor: Visitor,
    _data: PhantomData<&'a T>,
}

impl<'a, N: RealField, T, Visitor: FnMut(&T, &T) -> VisitStatus> AABBSetsInterferencesVisitor<'a, N, T, Visitor> {
    /// Creates a new `AABBSetsInterferencesVisitor`.
    #[inline]
    pub fn new(
        tolerence: N,
        ls_m2: &'a Isometry<N>,
        ls_m2_abs_rot: &'a Matrix<N>,
        visitor: Visitor,
    ) -> Self
    {
        Self {
            tolerence,
            ls_m2,
            ls_m2_abs_rot,
            visitor,
            _data: PhantomData
        }
    }
}

impl<'a, N: RealField, T, Visitor: FnMut(&T, &T) -> VisitStatus> SimultaneousVisitor<T, AABB<N>>
    for AABBSetsInterferencesVisitor<'a, N, T, Visitor>
{
    #[inline]
    fn visit(
        &mut self,
        left_bv: &AABB<N>,
        left_data: Option<&T>,
        right_bv: &AABB<N>,
        right_data: Option<&T>,
    ) -> VisitStatus
    {
        let ls_right_bv = AABB::from_half_extents(
            self.ls_m2 * right_bv.center(),
            self.ls_m2_abs_rot * right_bv.half_extents() + Vector::repeat(self.tolerence),
        );

        if left_bv.intersects(&ls_right_bv) {
            if let (Some(a), Some(b)) = (left_data, right_data) {
                (self.visitor)(a, b)
            } else {
                VisitStatus::Continue
            }
        } else {
            VisitStatus::Stop
        }
    }
}
