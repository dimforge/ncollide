use std::marker::PhantomData;
use na::{Identity, Translate, Translation};
use na;
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use shape::{Shape, CompositeShape};
use query::{Proximity, PointQuery};
use query::proximity_internal;
use math::{Point, Vector, Isometry};

/// Proximity between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn composite_shape_against_shape<P, M, G1: ?Sized>(m1: &M, g1: &G1,
                                                       m2: &M, g2: &Shape<P, M>,
                                                       margin: <P::Vect as Vector>::Scalar)
                                                       -> Proximity
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P> + Translation<P::Vect>,
          G1: CompositeShape<P, M> {
    assert!(margin >= na::zero(), "The proximity margin must be positive or null.");

    let mut cost_fn = CompositeShapeAgainstAnyInterfCostFn::new(m1, g1, m2, g2, margin);

    match g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res) {
        None       => Proximity::Disjoint,
        Some(prox) => prox
    }
}

/// Proximity between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn shape_against_composite_shape<P, M, G2: ?Sized>(m1: &M, g1: &Shape<P, M>,
                                                       m2: &M, g2: &G2,
                                                       margin: <P::Vect as Vector>::Scalar)
                                                       -> Proximity
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P> + Translation<P::Vect>,
          G2: CompositeShape<P, M> {
    composite_shape_against_shape(m2, g2, m1, g1, margin)
}

struct CompositeShapeAgainstAnyInterfCostFn<'a, P: 'a + Point, M: 'a, G1: ?Sized + 'a> {
    msum_shift:  P::Vect,
    msum_margin: P::Vect,

    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a Shape<P, M>,
    margin: <P::Vect as Vector>::Scalar,

    found_intersection: bool,

    point_type: PhantomData<P>
}

impl<'a, P, M, G1: ?Sized> CompositeShapeAgainstAnyInterfCostFn<'a, P, M, G1>
    where P:  Point,
          M:  Isometry<P>,
          G1: CompositeShape<P, M> {
    pub fn new(m1: &'a M, g1: &'a G1, m2: &'a M, g2: &'a Shape<P, M>, margin: <P::Vect as Vector>::Scalar)
               -> CompositeShapeAgainstAnyInterfCostFn<'a, P, M, G1> {

        let ls_m2    = na::inverse(m1).expect("The transformation `m1` must be inversible.") * *m2;
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyInterfCostFn {
            msum_shift:         -ls_aabb2.center().to_vector(),
            msum_margin:        ls_aabb2.half_extents(),
            m1:                 m1,
            g1:                 g1,
            m2:                 m2,
            g2:                 g2,
            margin:             margin,
            found_intersection: false,
            point_type:         PhantomData
        }
    }
}

impl<'a, P, M, G1: ?Sized> BVTCostFn<<P::Vect as Vector>::Scalar, usize, AABB<P>>
for CompositeShapeAgainstAnyInterfCostFn<'a, P, M, G1>
    where P:  Point,
          P::Vect: Translate<P>,
          M:  Isometry<P> + Translation<P::Vect>,
          G1: CompositeShape<P, M> {
    type UserData = Proximity;

    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<P>) -> Option<<P::Vect as Vector>::Scalar> {
        // No need to continue if some parts intersect.
        if self.found_intersection {
            return None;
        }

        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(*bv.mins() + self.msum_shift + (-self.msum_margin),
                             *bv.maxs() + self.msum_shift + self.msum_margin);

        // Compute the distance to the origin.
        let distance = msum.distance_to_point(&Identity::new(), &na::origin(), true);
        if distance <= self.margin {
            Some(distance)
        }
        else {
            None
        }
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(<P::Vect as Vector>::Scalar, Proximity)> {
        let mut res = None;

        self.g1.map_transformed_part_at(*b, self.m1, &mut |m1, g1| {
            res = match proximity_internal::proximity_internal(m1, g1, self.m2, self.g2, self.margin) {
                Proximity::Disjoint     => None,
                Proximity::WithinMargin => Some((self.margin, Proximity::WithinMargin)),
                Proximity::Intersecting => {
                    self.found_intersection = true;
                    Some((na::zero(), Proximity::Intersecting))
                }
            }
        });

        res
    }
}
