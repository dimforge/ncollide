use na::{self, Real};

use bounding_volume::AABB;
use math::{Isometry, Point, Vector};
use partitioning::BVTCostFn;
use query::{time_of_impact_internal, Ray, RayCast};
use shape::{CompositeShape, Shape};
use utils::IsometryOps;

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn composite_shape_against_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N>
where
    N: Real,
    G1: CompositeShape<N>,
{
    let mut cost_fn = CompositeShapeAgainstAnyTOICostFn::new(m1, vel1, g1, m2, vel2, g2);

    g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn shape_against_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: Real,
    G2: CompositeShape<N>,
{
    composite_shape_against_shape(m2, vel2, g2, m1, vel1, g1)
}

struct CompositeShapeAgainstAnyTOICostFn<'a, N: 'a + Real, G1: ?Sized + 'a> {
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

impl<'a, N, G1: ?Sized> CompositeShapeAgainstAnyTOICostFn<'a, N, G1>
where
    N: Real,
    G1: CompositeShape<N>,
{
    pub fn new(
        m1: &'a Isometry<N>,
        vel1: &'a Vector<N>,
        g1: &'a G1,
        m2: &'a Isometry<N>,
        vel2: &'a Vector<N>,
        g2: &'a Shape<N>,
    ) -> CompositeShapeAgainstAnyTOICostFn<'a, N, G1> {
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyTOICostFn {
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

impl<'a, N, G1: ?Sized> BVTCostFn<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyTOICostFn<'a, N, G1>
where
    N: Real,
    G1: CompositeShape<N>,
{
    type UserData = N;

    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<N>) -> Option<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the TOI.
        msum.toi_with_ray(&Isometry::identity(), &self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let mut res = None;

        self.g1
            .map_transformed_part_at(*b, self.m1, &mut |_, m1, g1| {
                res = time_of_impact_internal::time_of_impact(
                    m1, self.vel1, g1, self.m2, self.vel2, self.g2,
                ).map(|toi| (toi, toi))
            });

        res
    }
}
