use std::marker::PhantomData;

use alga::linear::Translation;
use math::{Point, Isometry};
use geometry::shape::{Shape, Ball};
use geometry::query::Proximity;
use geometry::query::proximity_internal;
use narrow_phase::{ProximityDetector, ProximityDispatcher};


/// Proximity detector between two balls.
pub struct BallBallProximityDetector<P: Point, M> {
    proximity: Proximity,
    pt_type:   PhantomData<P>, // FIXME: can we avoid this?
    mat_type:  PhantomData<M>  // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallBallProximityDetector<P, M> {
    fn clone(&self) -> BallBallProximityDetector<P, M> {
        BallBallProximityDetector {
            proximity: self.proximity,
            pt_type:   PhantomData,
            mat_type:  PhantomData
        }
    }
}

impl<P: Point, M> BallBallProximityDetector<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallProximityDetector<P, M> {
        BallBallProximityDetector {
            proximity: Proximity::Disjoint,
            pt_type:   PhantomData,
            mat_type:  PhantomData
        }
    }
}

impl<P: Point, M: Isometry<P>> ProximityDetector<P, M> for BallBallProximityDetector<P, M> {
    fn update(&mut self, _: &ProximityDispatcher<P, M>,
              ma: &M, a: &Shape<P, M>,
              mb: &M, b: &Shape<P, M>,
              margin: P::Real)
              -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<P::Real>>(),
                                     b.as_shape::<Ball<P::Real>>()) {
            self.proximity = proximity_internal::ball_against_ball(
                &P::from_coordinates(ma.translation().to_vector()),
                a,
                &P::from_coordinates(mb.translation().to_vector()),
                b,
                margin);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn proximity(&self) -> Proximity {
        self.proximity
    }
}
