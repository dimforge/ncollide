use na::Translate;
use na;
use shape::{Shape, ConcaveShape};
use bounding_volume::{HasAABB, AABB};
use ray::{LocalRayCast, Ray};
use partitioning::BVTCostFn;
use geometry::time_of_impact_internal;
use math::{Scalar, Point, Vect, Isometry, HasInertiaMatrix};

/// Time Of Impact of a composite shape with any other shape, under translational movement.
pub fn concave_shape_against_shape<N, P, V, M, I, G1, G2>(m1: &M, dir1: &V, g1: &G1,
                                                          m2: &M, dir2: &V, g2: &G2)
                                                          -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + HasInertiaMatrix<I>,
          M:  Isometry<N, P, V>,
          I:  Send + Sync + Clone,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    let mut cost_fn = ConcaveShapeAgainstShapeTOICostFn::new(m1, dir1, g1, m2, dir2, g2);

    g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
}

/// Time Of Impact of any shape with a composite shape, under translational movement.
pub fn shape_against_concave_shape<N, P, V, M, I, G1, G2>(m1: &M, dir1: &V, g1: &G1,
                                                          m2: &M, dir2: &V, g2: &G2)
                                                          -> Option<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + HasInertiaMatrix<I>,
          M:  Isometry<N, P, V>,
          I:  Send + Sync + Clone,
          G1: Shape<N, P, V, M>,
          G2: ConcaveShape<N, P, V, M> {
    concave_shape_against_shape(m2, dir2, g2, m1, dir1, g1)
}

struct ConcaveShapeAgainstShapeTOICostFn<'a, P, V: 'a, M: 'a, G1: 'a, G2: 'a> {
    msum_shift:  V,
    msum_margin: V,
    ray:         Ray<P, V>,

    m1:   &'a M,
    dir1: &'a V,
    g1:   &'a G1,
    m2:   &'a M,
    dir2: &'a V,
    g2:   &'a G2
}

impl<'a, N, P, V, M, I, G1, G2> ConcaveShapeAgainstShapeTOICostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + HasInertiaMatrix<I>,
          M:  Isometry<N, P, V>,
          I:  Send + Sync + Clone,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    pub fn new(m1: &'a M, dir1: &'a V, g1: &'a G1, m2: &'a M, dir2: &'a V, g2: &'a G2)
        -> ConcaveShapeAgainstShapeTOICostFn<'a, P, V, M, G1, G2> {

        let ls_m2 = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        ConcaveShapeAgainstShapeTOICostFn {
            msum_shift:  -ls_aabb2.center().to_vec(),
            msum_margin: ls_aabb2.half_extents(),
            ray:         Ray::new(na::orig(), *dir2 - *dir1),
            m1:          m1,
            dir1:        dir1,
            g1:          g1,
            m2:          m2,
            dir2:        dir2,
            g2:          g2
        }
    }
}

impl<'a, N, P, V, M, I, G1, G2> BVTCostFn<N, uint, AABB<P>, N>
for ConcaveShapeAgainstShapeTOICostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + HasInertiaMatrix<I>,
          M:  Isometry<N, P, V>,
          I:  Send + Sync + Clone,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<P>) -> Option<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(*bv.mins() + self.msum_shift + (-self.msum_margin),
                             *bv.maxs() + self.msum_shift + self.msum_margin);

        // Compute the TOI.
        msum.toi_with_ray(&self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, N)> {
        self.g1.map_transformed_part_at(self.m1, *b, |m1, g1|
            time_of_impact_internal::shape_against_shape(m1, self.dir1, g1, self.m2, self.dir2, self.g2)
            .map(|toi| (toi, toi))
        )
    }
}
