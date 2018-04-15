use na::Real;
use math::{Isometry, Point};
use shape::{Ball, Shape};
use query::Proximity;
use query::proximity_internal;
use pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};

/// Proximity detector between two balls.
pub struct BallBallProximityDetector {
    proximity: Proximity,
}

impl Clone for BallBallProximityDetector {
    fn clone(&self) -> BallBallProximityDetector {
        BallBallProximityDetector {
            proximity: self.proximity,
        }
    }
}

impl BallBallProximityDetector {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallProximityDetector {
        BallBallProximityDetector {
            proximity: Proximity::Disjoint,
        }
    }
}

impl<N: Real> ProximityDetector<N> for BallBallProximityDetector {
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
