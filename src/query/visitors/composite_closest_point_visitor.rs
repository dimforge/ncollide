use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{PointProjection, PointQuery};
use crate::shape::CompositeShape;
use na::{self, RealField};

/// Best-first traversal visitor for computing the point closest to a composite shape.
pub struct CompositeClosestPointVisitor<'a, N: 'a + RealField + Copy, S: 'a + CompositeShape<N>> {
    shape: &'a S,
    point: &'a Point<N>,
    solid: bool,
}

impl<'a, N: RealField + Copy, S: CompositeShape<N>> CompositeClosestPointVisitor<'a, N, S> {
    /// Initializes a visitor that allows the computation of the point closest to `point` on `shape`.
    pub fn new(shape: &'a S, point: &'a Point<N>, solid: bool) -> Self {
        CompositeClosestPointVisitor {
            shape,
            point,
            solid,
        }
    }
}

impl<'a, N: RealField + Copy, S: CompositeShape<N> + PointQuery<N>> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeClosestPointVisitor<'a, N, S>
{
    type Result = PointProjection<N>;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        let dist = aabb.distance_to_point(&Isometry::identity(), self.point, true);

        let mut res = BestFirstVisitStatus::Continue {
            cost: dist,
            result: None,
        };

        if let Some(b) = data {
            if dist < best {
                self.shape
                    .map_part_at(*b, &Isometry::identity(), &mut |objm, obj| {
                        let proj = obj.project_point(objm, self.point, self.solid);

                        res = BestFirstVisitStatus::Continue {
                            cost: na::distance(self.point, &proj.point),
                            result: Some(proj),
                        };
                    });
            }
        }

        res
    }
}
