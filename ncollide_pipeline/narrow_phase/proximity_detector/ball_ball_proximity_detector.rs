use std::marker::PhantomData;

use alga::linear::Translation;
use math::{Isometry, Point};
use geometry::shape::{Ball, Shape};
use geometry::query::Proximity;
use geometry::query::proximity_internal;
use narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Proximity detector between two balls.
pub struct BallBallProximityDetector<N: Real, M> {
    proximity: Proximity,
    pt_type: PhantomData<P>,  // FIXME: can we avoid this?
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<N: Real, M> Clone for BallBallProximityDetector<P, M> {
    fn clone(&self) -> BallBallProximityDetector<P, M> {
        BallBallProximityDetector {
            proximity: self.proximity,
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

impl<N: Real, M> BallBallProximityDetector<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallProximityDetector<P, M> {
        BallBallProximityDetector {
            proximity: Proximity::Disjoint,
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

impl<N: Real> ProximityDetector<P, M> for BallBallProximityDetector<P, M> {
    fn update(
        &mut self,
        _: &ProximityDispatcher<P, M>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<N>>(), b.as_shape::<Ball<N>>()) {
            self.proximity = proximity_internal::ball_against_ball(
                &Point::from_coordinates(ma.translation().to_vector()),
                a,
                &Point::from_coordinates(mb.translation().to_vector()),
                b,
                margin,
            );

            true
        } else {
            false
        }
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.proximity
    }
}
