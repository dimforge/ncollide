use crate::bounding_volume::{BoundingSphere, AABB};
use crate::interpolation::{RigidMotion, RigidMotionComposition};
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{self, TOIDispatcher, TOI};
use crate::shape::{Ball, CompositeShape, Shape};
use na::{self, RealField};

/// Time Of Impact of a composite shape with any other shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_composite_shape_shape<N, G1>(
    dispatcher: &dyn TOIDispatcher<N>,
    motion1: &dyn RigidMotion<N>,
    g1: &G1,
    motion2: &dyn RigidMotion<N>,
    g2: &dyn Shape<N>,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
{
    let mut visitor = CompositeShapeAgainstAnyNonlinearTOIVisitor::new(
        dispatcher,
        motion1,
        g1,
        motion2,
        g2,
        max_toi,
        target_distance,
    );

    g1.bvh().best_first_search(&mut visitor).map(|res| res.1)
}

/// Time Of Impact of any shape with a composite shape, under a rigid motion (translation + rotation).
pub fn nonlinear_time_of_impact_shape_composite_shape<N, G2>(
    dispatcher: &dyn TOIDispatcher<N>,
    motion1: &dyn RigidMotion<N>,
    g1: &dyn Shape<N>,
    motion2: &dyn RigidMotion<N>,
    g2: &G2,
    max_toi: N,
    target_distance: N,
) -> Option<TOI<N>>
where
    N: RealField,
    G2: ?Sized + CompositeShape<N>,
{
    nonlinear_time_of_impact_composite_shape_shape(
        dispatcher,
        motion2,
        g2,
        motion1,
        g1,
        max_toi,
        target_distance,
    )
}

struct CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N: 'a + RealField, G1: ?Sized + 'a> {
    dispatcher: &'a dyn TOIDispatcher<N>,
    sphere2: BoundingSphere<N>,
    max_toi: N,
    target_distance: N,

    motion1: &'a dyn RigidMotion<N>,
    g1: &'a G1,
    motion2: &'a dyn RigidMotion<N>,
    g2: &'a dyn Shape<N>,
}

impl<'a, N, G1> CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
{
    pub fn new(
        dispatcher: &'a dyn TOIDispatcher<N>,
        motion1: &'a dyn RigidMotion<N>,
        g1: &'a G1,
        motion2: &'a dyn RigidMotion<N>,
        g2: &'a dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1> {
        CompositeShapeAgainstAnyNonlinearTOIVisitor {
            dispatcher,
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

impl<'a, N, G1> BestFirstVisitor<N, usize, AABB<N>>
    for CompositeShapeAgainstAnyNonlinearTOIVisitor<'a, N, G1>
where
    N: RealField,
    G1: ?Sized + CompositeShape<N>,
{
    type Result = TOI<N>;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        bv: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        let sphere1 = bv.bounding_sphere();
        let ball1 = Ball::new(sphere1.radius());
        let ball2 = Ball::new(self.sphere2.radius());
        let motion1 = self.motion1.prepend_translation(sphere1.center().coords);
        let motion2 = self
            .motion2
            .prepend_translation(self.sphere2.center().coords);

        if let Some(toi) = query::nonlinear_time_of_impact_ball_ball(
            &motion1,
            &ball1,
            &motion2,
            &ball2,
            self.max_toi,
            self.target_distance,
        ) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi.toi,
                result: None,
            };

            if let Some(b) = data {
                if toi.toi < best {
                    self.g1
                        .map_part_at(*b, &Isometry::identity(), &mut |m1, g1| {
                            let motion1 = self.motion1.prepend_transformation(*m1);

                            if let Some(toi) = self
                                .dispatcher
                                .nonlinear_time_of_impact(
                                    self.dispatcher,
                                    &motion1,
                                    g1,
                                    self.motion2,
                                    self.g2,
                                    self.max_toi,
                                    self.target_distance,
                                )
                                .unwrap_or(None)
                            {
                                res = BestFirstVisitStatus::Continue {
                                    cost: toi.toi,
                                    result: Some(toi),
                                }
                            }
                        });
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}
