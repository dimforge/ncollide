use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{self, ClosestPoints, PointQuery};
use crate::shape::{CompositeShape, Shape};
use na::{self, RealField};

/// Closest points between a composite shape and any other shape.
pub fn closest_points_composite_shape_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &dyn Shape<N>,
    margin: N,
) -> ClosestPoints<N>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstShapeClosestPointsVisitor::new(m1, g1, m2, g2, margin);

    g1.bvh()
        .best_first_search(&mut visitor)
        .expect("The composite shape must not be empty.")
        .1
}

/// Closest points between a shape and a composite shape.
pub fn closest_points_shape_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
) -> ClosestPoints<N>
where
    N: RealField,
    G2: CompositeShape<N>,
{
    let mut res = closest_points_composite_shape_shape(m2, g2, m1, g1, margin);
    res.flip();
    res
}

struct CompositeShapeAgainstShapeClosestPointsVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a> {
    msum_shift: Vector<N>,
    msum_margin: Vector<N>,
    margin: N,

    m1: &'a Isometry<N>,
    g1: &'a G1,
    m2: &'a Isometry<N>,
    g2: &'a dyn Shape<N>,
}

impl<'a, N, G1: ?Sized> CompositeShapeAgainstShapeClosestPointsVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    pub fn new(
        m1: &'a Isometry<N>,
        g1: &'a G1,
        m2: &'a Isometry<N>,
        g2: &'a dyn Shape<N>,
        margin: N,
    ) -> CompositeShapeAgainstShapeClosestPointsVisitor<'a, N, G1> {
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstShapeClosestPointsVisitor {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            margin: margin,
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
        }
    }
}

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstShapeClosestPointsVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = ClosestPoints<N>;

    fn visit(
        &mut self,
        best: N,
        bv: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        let dist = msum.distance_to_point(&Isometry::identity(), &Point::origin(), true);

        let mut res = BestFirstVisitStatus::Continue {
            cost: dist,
            result: None,
        };

        if let Some(b) = data {
            if dist < best {
                self.g1.map_part_at(*b, self.m1, &mut |m1, g1| {
                    let pts = query::closest_points(m1, g1, self.m2, self.g2, self.margin);
                    match pts {
                        ClosestPoints::WithinMargin(ref p1, ref p2) => {
                            res = BestFirstVisitStatus::Continue {
                                cost: na::distance(p1, p2),
                                result: Some(pts),
                            }
                        }
                        ClosestPoints::Intersecting => {
                            res = BestFirstVisitStatus::ExitEarly(Some(pts))
                        }
                        ClosestPoints::Disjoint => {}
                    };
                });
            }
        }

        res
    }
}
