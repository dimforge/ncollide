use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{self, Ray, RayCast, TOI};
use crate::shape::{CompositeShape, Shape};
use crate::bounding_volume::bounding_volume::BoundingVolume;

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn time_of_impact_composite_shape_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &dyn Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyTOIVisitor::new(m1, vel1, g1, m2, vel2, g2, max_toi, target_distance);
    g1.bvh().best_first_search(&mut visitor).map(|res| res.1)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn time_of_impact_shape_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &dyn Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField,
    G2: CompositeShape<N>,
{
    time_of_impact_composite_shape_shape(m2, vel2, g2, m1, vel1, g1, max_toi, target_distance)
        .map(|toi| toi.swapped())
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
    g2: &'a dyn Shape<N>,
    max_toi: N,
    target_distance: N,
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
        g2: &'a dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
    {
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2).loosened(target_distance);

        CompositeShapeAgainstAnyTOIVisitor {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            ray: Ray::new(
                Point::origin(),
                m1.inverse_transform_vector(&(*vel2 - *vel1)),
            ),
            m1,
            vel1,
            g1,
            m2,
            vel2,
            g2,
            max_toi,
            target_distance,
        }
    }
}

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>> for CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = TOI<N>;

    #[inline]
    fn visit(&mut self, best: N, bv: &AABB<N>, data: Option<&usize>) -> BestFirstVisitStatus<N, Self::Result> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the TOI.
        if let Some(toi) = msum.toi_with_ray(&Isometry::identity(), &self.ray, true) {
            if toi > self.max_toi {
                return BestFirstVisitStatus::Stop;
            }

            let mut res = BestFirstVisitStatus::Continue { cost: toi, result: None };

            if let Some(b) = data {
                if toi < best {
                    self.g1.map_part_at(*b, self.m1, &mut |m1, g1| {
                        if let Some(toi) = query::time_of_impact(
                            m1, self.vel1, g1, self.m2, self.vel2, self.g2, self.max_toi, self.target_distance,
                        ) {
                            if toi.toi > self.max_toi {
                                res = BestFirstVisitStatus::Stop;
                            } else {
                                res = BestFirstVisitStatus::Continue { cost: toi.toi, result: Some(toi) }
                            }
                        }
                    });
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}
