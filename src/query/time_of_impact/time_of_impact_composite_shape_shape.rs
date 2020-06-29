use crate::bounding_volume::bounding_volume::BoundingVolume;
use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{Ray, RayCast, TOIDispatcher, TOI};
use crate::shape::{CompositeShape, Shape};
use na::{self, RealField};

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn time_of_impact_composite_shape_shape<N, G1: ?Sized>(
    dispatcher: &dyn TOIDispatcher<N>,
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
    let mut visitor = CompositeShapeAgainstAnyTOIVisitor::new(
        dispatcher,
        m1,
        vel1,
        g1,
        m2,
        vel2,
        g2,
        max_toi,
        target_distance,
    );
    g1.bvh().best_first_search(&mut visitor).map(|res| res.1)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn time_of_impact_shape_composite_shape<N, G2: ?Sized>(
    dispatcher: &dyn TOIDispatcher<N>,
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
    time_of_impact_composite_shape_shape(
        dispatcher,
        m2,
        vel2,
        g2,
        m1,
        vel1,
        g1,
        max_toi,
        target_distance,
    )
    .map(|toi| toi.swapped())
}

struct CompositeShapeAgainstAnyTOIVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a> {
    dispatcher: &'a dyn TOIDispatcher<N>,
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
        dispatcher: &'a dyn TOIDispatcher<N>,
        m1: &'a Isometry<N>,
        vel1: &'a Vector<N>,
        g1: &'a G1,
        m2: &'a Isometry<N>,
        vel2: &'a Vector<N>,
        g2: &'a dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> CompositeShapeAgainstAnyTOIVisitor<'a, N, G1> {
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2).loosened(target_distance);

        CompositeShapeAgainstAnyTOIVisitor {
            dispatcher,
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

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = TOI<N>;

    #[inline]
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

        // Compute the TOI.
        if let Some(toi) = msum.toi_with_ray(&Isometry::identity(), &self.ray, self.max_toi, true) {
            if toi > self.max_toi {
                return BestFirstVisitStatus::Stop;
            }

            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    self.g1.map_part_at(*b, self.m1, &mut |m1, g1| {
                        if let Some(toi) = self
                            .dispatcher
                            .time_of_impact(
                                self.dispatcher,
                                m1,
                                self.vel1,
                                g1,
                                self.m2,
                                self.vel2,
                                self.g2,
                                self.max_toi,
                                self.target_distance,
                            )
                            .unwrap_or(None)
                        {
                            if toi.toi > self.max_toi {
                                res = BestFirstVisitStatus::Stop;
                            } else {
                                res = BestFirstVisitStatus::Continue {
                                    cost: toi.toi,
                                    result: Some(toi),
                                }
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
