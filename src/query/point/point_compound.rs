use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor, BVH};
use crate::query::{visitors::CompositePointContainmentTest, PointProjection, PointQuery};
use crate::shape::{CompositeShape, Compound, FeatureId};
use na::{self, RealField};

impl<N: RealField + Copy> PointQuery<N> for Compound<N> {
    // XXX: if solid == false, this might return internal projection.
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = CompoundPointProjVisitor {
            compound: self,
            point: &ls_pt,
            solid: solid,
        };

        let mut proj = self.bvt().best_first_search(&mut visitor).unwrap().1;
        proj.point = m * proj.point;

        proj
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        _: &Isometry<N>,
        _: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        // XXX Properly propagate the feature id.
        unimplemented!()
        // (self.project_point(m, point), FeatureId::Unknown)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut visitor = CompositePointContainmentTest {
            shape: self,
            point: &ls_pt,
            found: false,
        };

        self.bvt().visit(&mut visitor);

        visitor.found
    }
}

/*
 * Visitors
 */
struct CompoundPointProjVisitor<'a, N: 'a + RealField + Copy> {
    compound: &'a Compound<N>,
    point: &'a Point<N>,
    solid: bool,
}

impl<'a, N: RealField + Copy> BestFirstVisitor<N, usize, AABB<N>>
    for CompoundPointProjVisitor<'a, N>
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
                self.compound
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
