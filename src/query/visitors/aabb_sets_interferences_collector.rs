use bounding_volume::{BoundingVolume, AABB};
use math::{Isometry, Matrix};
use na::Real;
use partitioning::{SimultaneousVisitor, VisitStatus};

/// Spatial partitioning data structure visitor collecting interferences with a given bounding volume.
pub struct AABBSetsInterferencesCollector<'a, N: 'a + Real, T: 'a> {
    /// The transform from the local-space of the second bounding volumes to the local space of the first.
    pub ls_m2: &'a Isometry<N>,
    /// The absolute value of the rotation matrix representing `ls_m2.rotation`.
    ///
    /// Equals to `ls_m2.rotation.to_rotation.matrix().matrix().abs()`.
    pub ls_m2_abs_rot: &'a Matrix<N>,
    /// The data contained by the nodes with bounding volumes intersecting `self.bv`.
    pub collector: &'a mut Vec<(T, T)>,
}

impl<'a, N: Real, T> AABBSetsInterferencesCollector<'a, N, T> {
    /// Creates a new `AABBSetsInterferencesCollector`.
    #[inline]
    pub fn new(
        ls_m2: &'a Isometry<N>,
        ls_m2_abs_rot: &'a Matrix<N>,
        collector: &'a mut Vec<(T, T)>,
    ) -> AABBSetsInterferencesCollector<'a, N, T>
    {
        AABBSetsInterferencesCollector {
            ls_m2,
            ls_m2_abs_rot,
            collector,
        }
    }
}

impl<'a, N: Real, T: Clone> SimultaneousVisitor<T, AABB<N>>
    for AABBSetsInterferencesCollector<'a, N, T>
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
            self.ls_m2_abs_rot * right_bv.half_extents(),
        );

        if left_bv.intersects(&ls_right_bv) {
            if let (Some(a), Some(b)) = (left_data, right_data) {
                self.collector.push((a.clone(), b.clone()))
            }

            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}
