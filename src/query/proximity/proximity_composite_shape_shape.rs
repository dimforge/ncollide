use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{self, PointQuery, Proximity};
use crate::shape::{CompositeShape, Shape};

/// Proximity between a composite shape (`Mesh`, `Compound`) and any other shape.
pub fn proximity_composite_shape_shape<N: RealField, G1: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &Shape<N>,
    margin: N,
) -> Proximity
where
    G1: CompositeShape<N>,
{
    assert!(
        margin >= na::zero(),
        "The proximity margin must be positive or null."
    );

    let mut visitor = CompositeShapeAgainstAnyInterfVisitor::new(m1, g1, m2, g2, margin);

    match g1.bvh().best_first_search(&mut visitor) {
        None => Proximity::Disjoint,
        Some(prox) => prox,
    }
}

/// Proximity between a shape and a composite (`Mesh`, `Compound`) shape.
pub fn proximity_shape_composite_shape<N: RealField, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    g2: &G2,
    margin: N,
) -> Proximity
where
    G2: CompositeShape<N>,
{
    proximity_composite_shape_shape(m2, g2, m1, g1, margin)
}

struct CompositeShapeAgainstAnyInterfVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a> {
    msum_shift: Vector<N>,
    msum_margin: Vector<N>,

    m1: &'a Isometry<N>,
    g1: &'a G1,
    m2: &'a Isometry<N>,
    g2: &'a Shape<N>,
    margin: N,
}

impl<'a, N: RealField, G1: ?Sized> CompositeShapeAgainstAnyInterfVisitor<'a, N, G1>
where G1: CompositeShape<N>
{
    pub fn new(
        m1: &'a Isometry<N>,
        g1: &'a G1,
        m2: &'a Isometry<N>,
        g2: &'a Shape<N>,
        margin: N,
    ) -> CompositeShapeAgainstAnyInterfVisitor<'a, N, G1>
    {
        let ls_m2 = m1.inverse() * m2.clone();
        let ls_aabb2 = g2.aabb(&ls_m2);

        CompositeShapeAgainstAnyInterfVisitor {
            msum_shift: -ls_aabb2.center().coords,
            msum_margin: ls_aabb2.half_extents(),
            m1: m1,
            g1: g1,
            m2: m2,
            g2: g2,
            margin: margin,
        }
    }
}

impl<'a, N: RealField, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyInterfVisitor<'a, N, G1>
where G1: CompositeShape<N>
{
    type Result = Proximity;

    fn visit_bv(&mut self, bv: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        // Compute the minkowski sum of the two AABBs.
        let msum = AABB::new(
            *bv.mins() + self.msum_shift + (-self.msum_margin),
            *bv.maxs() + self.msum_shift + self.msum_margin,
        );

        // Compute the distance to the origin.
        let distance = msum.distance_to_point(&Isometry::identity(), &Point::origin(), true);
        BestFirstBVVisitStatus::ContinueWithCost(distance)
    }

    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, Proximity> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.g1
            .map_part_at(*b, self.m1, &mut |m1, g1| {
                res = match query::proximity(
                    m1,
                    g1,
                    self.m2,
                    self.g2,
                    self.margin,
                ) {
                    Proximity::Disjoint => BestFirstDataVisitStatus::Continue,
                    Proximity::WithinMargin => BestFirstDataVisitStatus::ContinueWithResult(
                        self.margin,
                        Proximity::WithinMargin,
                    ),
                    Proximity::Intersecting => {
                        BestFirstDataVisitStatus::ExitEarlyWithResult(Proximity::Intersecting)
                    }
                }
            });

        res
    }
}
