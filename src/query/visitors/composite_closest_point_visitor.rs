use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use na::{self, RealField};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{PointProjection, PointQuery};
use crate::shape::CompositeShape;

/// Best-fisrt traversal visitor for computin the point closest to a composite shape.
pub struct CompositeClosestPointVisitor<'a, N: 'a + RealField, S: 'a + CompositeShape<N>> {
    shape: &'a S,
    point: &'a Point<N>,
    solid: bool,
}

impl<'a, N: RealField, S: CompositeShape<N>> CompositeClosestPointVisitor<'a, N, S> {
    /// Initializes a visitor that allows the computation of the point closest to `point` on `shape`.
    pub fn new(shape: &'a S, point: &'a Point<N>, solid: bool) -> Self {
        CompositeClosestPointVisitor {
            shape,
            point,
            solid,
        }
    }
}

impl<'a, N: RealField, S: CompositeShape<N> + PointQuery<N>> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeClosestPointVisitor<'a, N, S>
{
    type Result = PointProjection<N>;

    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        BestFirstBVVisitStatus::ContinueWithCost(aabb.distance_to_point(
            &Isometry::identity(),
            self.point,
            true,
        ))
    }

    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, PointProjection<N>> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.shape.map_part_at(*b, &Isometry::identity(), &mut |objm, obj| {
            let proj = obj.project_point(objm, self.point, self.solid);
            res = BestFirstDataVisitStatus::ContinueWithResult(
                na::distance(self.point, &proj.point),
                proj,
            );
        });

        res
    }
}
