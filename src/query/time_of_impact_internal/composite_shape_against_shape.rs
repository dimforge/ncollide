use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{Ray, RayCast, time_of_impact, time_of_impact_and_normal};
use crate::shape::{CompositeShape, Shape};

/// Time of impact of a composite shape with any other shape, under translational movement.
pub fn composite_shape_against_shape<N, G1: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<N>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyTOIVisitor::new(m1, vel1, g1, m2, vel2, g2);

    g1.bvh().best_first_search(&mut visitor)
}

/// Time of impact of any shape with a composite shape, under translational movement.
pub fn shape_against_composite_shape<N, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<N>
where
    N: RealField,
    G2: CompositeShape<N>,
{
    composite_shape_against_shape(m2, vel2, g2, m1, vel1, g1)
}

/// Time of impact and contact normal of a composite shape with any other shape,
/// under translational movement.
pub fn composite_shape_against_shape_with_normal<N, G1: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &G1,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &Shape<N>,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyTOIVisitorWithNormal::new(m1, vel1, g1, m2, vel2, g2);

    g1.bvh().best_first_search(&mut visitor)
}

/// Time of impact and contact normal of any shape with a composite shape, under
/// translational movement.
pub fn shape_against_composite_shape_with_normal<N, G2: ?Sized>(
    m1: &Isometry<N>,
    vel1: &Vector<N>,
    g1: &Shape<N>,
    m2: &Isometry<N>,
    vel2: &Vector<N>,
    g2: &G2,
) -> Option<(N, Vector<N>)>
where
    N: RealField,
    G2: CompositeShape<N>,
{
    composite_shape_against_shape_with_normal(m2, vel2, g2, m1, vel1, g1).map(|x| (x.0, -x.1))
}

macro_rules! impl_composite_shape_visitor {
    ($name:ident) => (
        struct $name<'a, N: 'a + RealField, G1: ?Sized + 'a> {
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

        impl<'a, N, G1: ?Sized> $name<'a, N, G1>
        where
            N: RealField,
            G1: CompositeShape<N>,
        {
            pub fn new(
                m1: &'a Isometry<N>,
                vel1: &'a Vector<N>,
                g1: &'a G1,
                m2: &'a Isometry<N>,
                vel2: &'a Vector<N>,
                g2: &'a Shape<N>,
            ) -> $name<'a, N, G1>
            {
                let ls_m2 = m1.inverse() * m2.clone();
                let ls_aabb2 = g2.aabb(&ls_m2);

                $name {
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
    )
}

impl_composite_shape_visitor!(CompositeShapeAgainstAnyTOIVisitor);
impl_composite_shape_visitor!(CompositeShapeAgainstAnyTOIVisitorWithNormal);

#[inline]
fn visit_bv<N: RealField>(bv: &AABB<N>, msum_shift: &Vector<N>, msum_margin: &Vector<N>, ray: &Ray<N>) -> BestFirstBVVisitStatus<N> {
    // Compute the minkowski sum of the two AABBs.
    let msum = AABB::new(
        *bv.mins() + msum_shift + (-msum_margin),
        *bv.maxs() + msum_shift + msum_margin,
    );

    // Compute the TOI.
    match msum.toi_with_ray(&Isometry::identity(), &ray, true) {
        Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
        None => BestFirstBVVisitStatus::Stop,
    }
}

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, bv: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        visit_bv(bv, &self.msum_shift, &self.msum_margin, &self.ray)
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, Self::Result> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.g1
            .map_part_at(*b, self.m1, &mut |m1, g1| {
                if let Some(toi) = time_of_impact(
                    m1, self.vel1, g1, self.m2, self.vel2, self.g2,
                ) {
                    res = BestFirstDataVisitStatus::ContinueWithResult(toi, toi)
                }
            });

        res
    }
}

impl<'a, N, G1: ?Sized> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyTOIVisitorWithNormal<'a, N, G1>
where
    N: RealField,
    G1: CompositeShape<N>,
{
    type Result = (N, Vector<N>);

    #[inline]
    fn visit_bv(&mut self, bv: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        visit_bv(bv, &self.msum_shift, &self.msum_margin, &self.ray)
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, Self::Result> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.g1
            .map_part_at(*b, self.m1, &mut |m1, g1| {
                if let Some((toi, normal)) = time_of_impact_and_normal(
                    m1, self.vel1, g1, self.m2, self.vel2, self.g2,
                ) {
                    res = BestFirstDataVisitStatus::ContinueWithResult(toi, (toi, normal))
                }
            });

        res
    }
}
