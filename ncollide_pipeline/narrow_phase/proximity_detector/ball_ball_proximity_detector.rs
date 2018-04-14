use std::marker::PhantomData;

use alga::linear::Translation;
use math::{Isometry, Point};
use shape::{Ball, Shape};
use query::Proximity;
use query::proximity_internal;
use pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Proximity detector between two balls.
pub struct BallBallProximityDetector<N> {
    proximity: Proximity,
    pt_type: PhantomData<P>,  // FIXME: can we avoid this?
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<N> Clone for BallBallProximityDetector<N> {
    fn clone(&self) -> BallBallProximityDetector<N> {
        BallBallProximityDetector {
            proximity: self.proximity,
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

impl<N> BallBallProximityDetector<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallProximityDetector<N> {
        BallBallProximityDetector {
            proximity: Proximity::Disjoint,
            pt_type: PhantomData,
            mat_type: PhantomData,
        }
    }
}

impl<N: Real> ProximityDetector<N> for BallBallProximityDetector<N> {
    fn update(
        &mut self,
        _: &ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        margin: N,
    ) -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<N>>(), b.as_shape::<Ball<N>>()) {
            self.proximity = proximity_internal::ball_against_ball(
                &Point::from_coordinates(ma.translation.vector),
                a,
                &Point::from_coordinates(mb.translation.vector),
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
