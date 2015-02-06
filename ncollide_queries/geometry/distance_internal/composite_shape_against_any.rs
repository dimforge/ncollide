use na::Translate;
use na;
use entities::bounding_volume::{HasAABB, AABB};
use entities::partitioning::BVTCostFn;
use entities::shape::CompositeShape;
use entities::inspection::Repr;
use geometry::distance_internal;
use point::LocalPointQuery;
use math::{Scalar, Point, Vect, Isometry};

/// Smallest distance between a composite shape and any other shape.
pub fn composite_shape_against_any<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: CompositeShape<N, P, V, M>,
          G2: Repr<N, P, V, M> + HasAABB<P, M> {
    let mut cost_fn = CompositeShapeAgainstAnyDistCostFn::new(m1, g1, m2, g2);

    g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res).expect("The composite shape must not be empty.")
}

/// Smallest distance between a shape and a composite shape.
pub fn any_against_composite_shape<N, P, V, M, G1: ?Sized, G2: ?Sized>(m1: &M, g1: &G1, m2: &M, g2: &G2) -> N
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: Repr<N, P, V, M> + HasAABB<P, M>,
          G2: CompositeShape<N, P, V, M> {
    composite_shape_against_any(m2, g2, m1, g1)
}

struct CompositeShapeAgainstAnyDistCostFn<'a, P, V: 'a, M: 'a, G1: ?Sized + 'a, G2: ?Sized + 'a> {
    msum_shift:  V,
    msum_margin: V,

    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a G2
}

#[old_impl_check]
impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized> CompositeShapeAgainstAnyDistCostFn<'a, P, V, M, G1, G2>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> ,
          M:  Isometry<N, P, V>,
          G1: CompositeShape<N, P, V, M>,
          G2: Repr<N, P, V, M> + HasAABB<P, M> {
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a G2)
        -> CompositeShapeAgainstAnyDistCostFn<'a, P, V, M, G1, G2> {

        let ls_m2 = na::inv(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyDistCostFn {
            msum_shift:  -ls_aabb2.center().to_vec(),
            msum_margin: ls_aabb2.half_extents(),
            m1:          m1,
            g1:          g1,
            m2:          m2,
            g2:          g2
        }
    }
}

impl<'a, N, P, V, M, G1: ?Sized, G2: ?Sized> BVTCostFn<N, usize, AABB<P>, N>
for CompositeShapeAgainstAnyDistCostFn<'a, P, V, M, G1, G2>
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

        // Compute the distance to the origin.
        Some(msum.distance_to_point(&na::orig()))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        let mut res = None;

        self.g1.map_transformed_part_at(self.m1, *b, &mut |m1, g1| {
            let dist = distance_internal::any_against_any(m1, g1, self.m2, self.g2);

            res = Some((dist, dist))
        });

        res
    }
}
