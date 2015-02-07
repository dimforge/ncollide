use na::Translate;
use na;
use math::{Scalar, Point, Vect, Isometry};
use entities::bounding_volume::{HasAABB, AABB};
use entities::partitioning::BVTCostFn;
use entities::inspection::Repr;
use entities::shape::CompositeShape;
use ray::{LocalRayCast, Ray};
use geometry::time_of_impact_internal;

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn composite_shape_against_any<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &M, vel1: &V, g1: &G1,
                                                                       m2: &M, vel2: &V, g2: &G2)
                                                                       -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: CompositeShape<N, P, V, M>,
          G2: Repr<N, P, V, M> + HasAABB<P, M> {
    let mut cost_fn = CompositeShapeAgainstAnyTOICostFn::new(m1, vel1, g1, m2, vel2, g2);

    g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn any_against_composite_shape<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &M, vel1: &V, g1: &G1,
                                                                       m2: &M, vel2: &V, g2: &G2)
                                                                       -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: Repr<N, P, V, M> + HasAABB<P, M>,
          G2: CompositeShape<N, P, V, M> {
    composite_shape_against_any(m2, vel2, g2, m1, vel1, g1)
}

struct CompositeShapeAgainstAnyTOICostFn<'a, P, V: 'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    msum_shift:  V,
    msum_margin: V,
    ray:         Ray<P, V>,

    m1:   &'a M,
    vel1: &'a V,
    g1:   &'a G1,
    m2:   &'a M,
    vel2: &'a V,
    g2:   &'a G2
}

#[old_impl_check]
impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized> CompositeShapeAgainstAnyTOICostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: CompositeShape<N, P, V, M>,
          G2: Repr<N, P, V, M> + HasAABB<P, M> {
    pub fn new(m1: &'a M, vel1: &'a V, g1: &'a G1, m2: &'a M, vel2: &'a V, g2: &'a G2)
        -> CompositeShapeAgainstAnyTOICostFn<'a, P, V, M, G1, G2> {

        let ls_m2 = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyTOICostFn {
            msum_shift:  -ls_aabb2.center().to_vec(),
            msum_margin: ls_aabb2.half_extents(),
            ray:         Ray::new(na::orig(), m1.inv_rotate(&(*vel2 - *vel1))),
            m1:          m1,
            vel1:        vel1,
            g1:          g1,
            m2:          m2,
            vel2:        vel2,
            g2:          g2
        }
    }
}

impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized> BVTCostFn<N, usize, AABB<P>, N>
for CompositeShapeAgainstAnyTOICostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: CompositeShape<N, P, V, M>,
          G2: Repr<N, P, V, M> + HasAABB<P, M> {
    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<P>) -> Option<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(*bv.mins() + self.msum_shift + (-self.msum_margin),
                             *bv.maxs() + self.msum_shift + self.msum_margin);

        // Compute the TOI.
        msum.toi_with_ray(&self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let mut res = None;

        self.g1.map_transformed_part_at(self.m1, *b, &mut |m1, g1|
            res = time_of_impact_internal::any_against_any(m1, self.vel1, g1,
                                                           self.m2, self.vel2, self.g2)
                  .map(|toi| (toi, toi))
        );

        res
    }
}
