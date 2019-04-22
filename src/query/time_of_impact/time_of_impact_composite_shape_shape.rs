use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{self, Ray, RayCast};
use crate::shape::{CompositeShape, Shape};

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn time_of_impact_composite_shape_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyTOIVisitor::new(m1, vel1, g1, m2, vel2, g2);

    g1.bvh().best_first_search(&mut visitor)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn time_of_impact_shape_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: RealField,
    G2: CompositeShape<N>,
{
    time_of_impact_composite_shape_shape(m2, vel2, g2, m1, vel1, g1)
}

struct CompositeShapeAgainstAnyTOIVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a> {
    msum_shift: Vector<N>,
    msum_margin: Vector<N>,
    ray: Ray<N>,

    m1: &'a Isometry<N>,
    vel1: &'a Vector<N>,
    g1: &'a G1,
    m2: &'a Isometry<N>,
    vel2: &'a Vector<N>,
    g2: &'a Shape<N>,
}

impl<'a, N, G1: ?Sized> CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    pub fn new(
        m1: &'a Isometry<N>,
        vel1: &'a Vector<N>,
        g1: &'a G1,
        m2: &'a Isometry<N>,
        vel2: &'a Vector<N>,
        g2: &'a Shape<N>,
    ) -> CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
    {
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyTOIVisitor {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            ray: Ray::new(
                Point::origin(),
                m1.inverse_transform_vector(&(*vel2 - *vel1)),
            ),
            m1: m1,
            vel1: vel1,
            g1: g1,
            m2: m2,
            vel2: vel2,
            g2: g2,
        }
    }
}

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, bv: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the TOI.
        match msum.toi_with_ray(&Isometry::identity(), &self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, N> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.g1
            .map_part_at(*b, self.m1, &mut |m1, g1| {
                if let Some(toi) = query::time_of_impact(
                    m1, self.vel1, g1, self.m2, self.vel2, self.g2,
                ) {
                    res = BestFirstDataVisitStatus::ContinueWithResult(toi, toi)
                }
            });

        res
    }
}
