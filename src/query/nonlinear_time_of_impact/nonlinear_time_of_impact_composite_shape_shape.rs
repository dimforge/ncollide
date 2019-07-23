use crate::bounding_volume::{AABB, BoundingSphere};
use crate::math::{Isometry, Point, Vector};
use na::{self, RealField};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{self, Ray, RayCast, NonlinearTOI};
use crate::shape::{CompositeShape, Shape, Ball};
use crate::interpolation::{RigidMotion, RigidMotionComposition};

/// Time Of Impact of a composite shape with any other shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_composite_shape_shape<N, G1>(
    motion1: &(impl RigidMotion<N> + ?Sized),
    g1: &G1,
    motion2: &(impl RigidMotion<N> + ?Sized),
    g2: &Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Option<NonlinearTOI<N>>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyNonlinearTOIVisitor::new(motion1, g1, motion2, g2, max_toi, target_distance);

    g1.bvh().best_first_search(&mut visitor)
}

/// Time Of Impact of any shape with a composite shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_shape_composite_shape<N, G2>(
    motion1: &(impl RigidMotion<N> + ?Sized),
    g1: &Shape<N>,
    motion2: &(impl RigidMotion<N> + ?Sized),
    g2: &G2,
    max_toi: N,
    target_distance: N,
) -> Option<NonlinearTOI<N>>
where
    N: RealField,
    G2: ?Sized + CompositeShape<N>,
{
    nonlinear_time_of_impact_composite_shape_shape(motion2, g2, motion1, g1, max_toi, target_distance)
}

struct CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a, M1: ?Sized, M2: ?Sized> {
    sphere2: BoundingSphere<N>,
    max_toi: N,
    target_distance: N,

    motion1: &'a M1,
    g1: &'a G1,
    motion2: &'a M2,
    g2: &'a Shape<N>,
}

impl<'a, N, G1, M1, M2> CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1, M1, M2>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
    M1: ?Sized + RigidMotion<N>,
    M2: ?Sized + RigidMotion<N>,
{
    pub fn new(
        motion1: &'a M1,
        g1: &'a G1,
        motion2: &'a M2,
        g2: &'a Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1, M1, M2>
    {
        CompositeShapeAgainstAnyNonlinearTOIVisitor {
            sphere2: g2.bounding_sphere(&Isometry::identity()),
            max_toi,
            target_distance,
            motion1,
            g1,
            motion2,
            g2,
        }
    }
}

impl<'a, N, G1, M1, M2> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1, M1, M2>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
    M1: ?Sized + RigidMotion<N>,
    M2: ?Sized + RigidMotion<N>,
{
    type Result = NonlinearTOI<N>;

    #[inline]
    fn visit_bv(&mut self, bv: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        let sphere1 = bv.bounding_sphere();
        let ball1 = Ball::new(sphere1.radius());
        let ball2 = Ball::new(self.sphere2.radius());
        let motion1 = self.motion1.prepend_translation(sphere1.center().coords);
        let motion2 = self.motion2.prepend_translation(self.sphere2.center().coords);

        if let Some(toi) = query::nonlinear_time_of_impact_ball_ball(
            &motion1, &ball1, &motion2, &ball2, self.max_toi, self.target_distance) {
            BestFirstBVVisitStatus::ContinueWithCost(toi.toi)
        } else {
            BestFirstBVVisitStatus::Stop
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, NonlinearTOI<N>> {
        let mut res = BestFirstDataVisitStatus::Continue;

        self.g1.map_part_at(*b, &Isometry::identity(), &mut |m1, g1| {
            let motion1 = self.motion1.prepend_transformation(*m1);

            // NOTE: we have to use a trait-object for `&motion1 as &RigidMotion<N>` to avoid infinite
            // compiler recursion when it monomorphizes query::nonlinear_time_of_impact.
            if let Some(toi) =
                query::nonlinear_time_of_impact(&motion1 as &RigidMotion<N>, g1, self.motion2, self.g2, self.max_toi, self.target_distance) {
                res = BestFirstDataVisitStatus::ContinueWithResult(toi.toi, toi)
            }
        });

        res
    }
}
