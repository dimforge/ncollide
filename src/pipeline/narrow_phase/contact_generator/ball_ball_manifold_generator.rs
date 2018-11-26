use bounding_volume::ConicalApproximation;
use math::{Isometry, Point};
use na::Real;
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use query::contacts_internal;
use query::{ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry, ContactPreprocessor};
use shape::{Ball, FeatureId, Shape};
use std::marker::PhantomData;
use utils::IdAllocator;

/// Collision detector between two balls.
#[derive(Clone)]
pub struct BallBallManifoldGenerator<N: Real> {
    phantom: PhantomData<N>,
}

impl<N: Real> BallBallManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallManifoldGenerator<N> {
        BallBallManifoldGenerator {
            phantom: PhantomData,
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for BallBallManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<N>>(), b.as_shape::<Ball<N>>()) {
            let center_a = Point::from_coordinates(ma.translation.vector);
            let center_b = Point::from_coordinates(mb.translation.vector);
            if let Some(contact) = contacts_internal::ball_against_ball(
                &center_a,
                a,
                &center_b,
                b,
                prediction.linear(),
            ) {
                let mut kinematic = ContactKinematic::new();
                kinematic.set_approx1(
                    FeatureId::Face(0),
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_approx2(
                    FeatureId::Face(0),
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_dilation1(a.radius());
                kinematic.set_dilation2(b.radius());

                let _ = manifold.push(contact, kinematic, Point::origin(), proc1, proc2, id_alloc);
            }

            true
        } else {
            false
        }
    }
}
