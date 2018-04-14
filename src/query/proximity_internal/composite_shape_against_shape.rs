use std::marker::PhantomData;

use alga::general::Id;
use na;

use bounding_volume::AABB;
use partitioning::BVTCostFn;
use shape::{CompositeShape, Shape};
use query::{PointQuery, Proximity};
use query::proximity_internal;
use math::{Isometry, Point};

/// Proximity between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn composite_shape_against_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &Shape<N>,
    margin: N,
) -> Proximity
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let mut cost_fn = CompositeShapeAgainstAnyInterfCostFn::new(m1, g1, m2, g2, margin);

    match g1.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res) {
        None => Proximity::Disjoint,
        Some(prox) => prox,
    }
}

/// Proximity between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn shape_against_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
) -> Proximity
where
    N: Real,
    M: Isometry<P>,
    G2: CompositeShape<N>,
{
    composite_shape_against_shape(m2, g2, m1, g1, margin)
}

struct CompositeShapeAgainstAnyInterfCostFn<'a, P: 'a + Point, M: 'a, G1: ?Sized + 'a> {
    msum_shift: Vector<N>,
    msum_margin: Vector<N>,

    m1: &'a M,
    g1: &'a G1,
    m2: &'a M,
    g2: &'a Shape<N>,
    margin: N,

    found_intersection: bool,

    point_type: PhantomData<P>,
}

impl<'a, N, G1: ?Sized> CompositeShapeAgainstAnyInterfCostFn<'a, N, G1>
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<N>,
{
    pub fn new(
        m1: &'a M,
        g1: &'a G1,
        m2: &'a M,
        g2: &'a Shape<N>,
        margin: N,
    ) -> CompositeShapeAgainstAnyInterfCostFn<'a, N, G1> {
        let ls_m2 = na::inverse(m1) * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyInterfCostFn {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
            margin: margin,
            found_intersection: false,
            point_type: PhantomData,
        }
    }
}

impl<'a, N, G1: ?Sized> BVTCostFn<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyInterfCostFn<'a, N, G1>
where
    N: Real,
    M: Isometry<P>,
    G1: CompositeShape<N>,
{
    type UserData = Proximity;

    #[inline]
    fn compute_bv_cost(&mut self, bv: &AABB<N>) -> Option<N> {
        // No need to continue if some parts intersect.
        if self.found_intersection {
            return None;
        }

        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the distance to the origin.
        let distance = msum.distance_to_point(&Isometry::identity(), &Point::origin(), true);
        if distance <= self.margin {
            Some(distance)
        } else {
            None
        }
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, Proximity)> {
        let mut res = None;

        self.g1
            .map_transformed_part_at(*b, self.m1, &mut |_, m1, g1| {
                res = match proximity_internal::proximity_internal(
                    m1,
                    g1,
                    self.m2,
                    self.g2,
                    self.margin,
                ) {
                    Proximity::Disjoint => None,
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
