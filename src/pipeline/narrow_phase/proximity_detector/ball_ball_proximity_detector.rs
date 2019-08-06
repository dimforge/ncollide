use crate::math::{Isometry, Point};
use na::RealField;
use crate::pipeline::narrow_phase::{ProximityDetector, ProximityDispatcher};
use crate::query::{self, Proximity};
use crate::shape::{Ball, Shape};

/// Proximity detector between two balls.
pub struct BallBallProximityDetector {
}

impl Clone for BallBallProximityDetector {
    fn clone(&self) -> BallBallProximityDetector {
        BallBallProximityDetector {
        }
    }
}

impl BallBallProximityDetector {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallProximityDetector {
        BallBallProximityDetector {
        }
    }
}

impl<N: RealField> ProximityDetector<N> for BallBallProximityDetector {
    fn update(
        &mut self,
        _: &dyn ProximityDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        margin: N,
    ) -> Option<Proximity>
    {
        let a = a.as_shape::<Ball<N>>()?;
        let b = b.as_shape::<Ball<N>>()?;
        Some(query::proximity_ball_ball(
            &Point::from(ma.translation.vector),
            a,
            &Point::from(mb.translation.vector),
            b,
            margin,
        ))
    }
}
