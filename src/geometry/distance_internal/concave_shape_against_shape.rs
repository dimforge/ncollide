use na::Translate;
use na;
use shape::{Shape, ConcaveShape};
use bounding_volume::{HasAABB, AABB};
use partitioning::BVTCostFn;
use geometry::distance_internal;
use point::LocalPointQuery;
use math::{Scalar, Point, Vect, Isometry};

/// Smallest distance between a concave shape and any other shape.
pub fn concave_shape_against_shape<N, P, V, M, G1, G2>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    let mut cost_fn = ConcaveShapeAgainstShapeDistCostFn::new(m1, g1, m2, g2);

    g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res).expect("The concave shape must not be empty.")
}

/// Smallest distance between a shape and a concave shape.
pub fn shape_against_concave_shape<N, P, V, M, G1, G2>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: Shape<N, P, V, M>,
          G2: ConcaveShape<N, P, V, M> {
    concave_shape_against_shape(m2, g2, m1, g1)
}

struct ConcaveShapeAgainstShapeDistCostFn<'a, P, V: 'a, M: 'a, G1: 'a, G2: 'a> {
    msum_shift:  V,
    msum_margin: V,

    m1:   &'a M,
    g1:   &'a G1,
    m2:   &'a M,
    g2:   &'a G2
}

impl<'a, N, P, V, M, G1, G2> ConcaveShapeAgainstShapeDistCostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a G2)
        -> ConcaveShapeAgainstShapeDistCostFn<'a, P, V, M, G1, G2> {

        let ls_m2 = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        ConcaveShapeAgainstShapeDistCostFn {
            msum_shift:  -ls_aabb2.center().to_vec(),
            msum_margin: ls_aabb2.half_extents(),
            m1:          m1,
            g1:          g1,
            m2:          m2,
            g2:          g2
        }
    }
}

impl<'a, N, P, V, M, G1, G2> BVTCostFn<N, uint, AABB<P>, N>
for ConcaveShapeAgainstShapeDistCostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: ConcaveShape<N, P, V, M>,
          G2: Shape<N, P, V, M> {
    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<P>) -> Option<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(*bv.mins() + self.msum_shift + (-self.msum_margin),
                             *bv.maxs() + self.msum_shift + self.msum_margin);

        // Compute the distance to the origin.
        Some(msum.distance_to_point(&na::orig()))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, N)> {
        self.g1.map_transformed_part_at(self.m1, *b, |m1, g1| {
            let dist = distance_internal::shape_against_shape(m1, g1, self.m2, self.g2);
            Some((dist, dist))
        })
    }
}
