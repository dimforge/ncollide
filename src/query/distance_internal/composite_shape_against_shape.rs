use std::marker::PhantomData;

use alga::general::Id;

use na;
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use shape::{CompositeShape, Shape};
use query::distance_internal;
use query::PointQuery;
use math::{Isometry, Point};

/// Smallest distance between a composite shape and any other shape.
pub fn composite_shape_against_shape<P, M, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &Shape<N>,
) -> N
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<P, M>,
{
    let mut cost_fn = CompositeShapeAgainstAnyDistCostFn::new(m1, g1, m2, g2);

    g1.bvt()
        .best_first_search(&mut cost_fn)
        .map(|(_, res)| res)
        .expect("The composite shape must not be empty.")
}

/// Smallest distance between a shape and a composite shape.
pub fn shape_against_composite_shape<P, M, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
) -> N
where
    N: Real,
    M: Isometry<P>,
    G2: CompositeShape<P, M>,
{
    composite_shape_against_shape(m2, g2, m1, g1)
}

struct CompositeShapeAgainstAnyDistCostFn<'a, P: 'a + Point, M: 'a, G1: ?Sized + 'a> {
    msum_shift: Vector<N>,
    msum_margin: Vector<N>,

    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a Shape<N>,

    point_type: PhantomData<P>,
}

impl<'a, P, M, G1: ?Sized> CompositeShapeAgainstAnyDistCostFn<'a, P, M, G1>
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<P, M>,
{
    pub fn new(
        m1: &'a M,
        g1: &'a G1,
        m2: &'a M,
        g2: &'a Shape<N>,
    ) -> CompositeShapeAgainstAnyDistCostFn<'a, P, M, G1> {
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyDistCostFn {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
            point_type: PhantomData,
        }
    }
}

impl<'a, P, M, G1: ?Sized> BVTCostFn<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyDistCostFn<'a, P, M, G1>
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<P, M>,
{
    type UserData = N;
    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<N>) -> Option<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the distance to the origin.
        Some(msum.distance_to_point(&Isometry::identity(), &Point::origin(), true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let mut res = None;

        self.g1
            .map_transformed_part_at(*b, self.m1, &mut |_, m1, g1| {
                let distance = distance_internal::distance(m1, g1, self.m2, self.g2);

                res = Some((distance, distance))
            });

        res
    }
}
