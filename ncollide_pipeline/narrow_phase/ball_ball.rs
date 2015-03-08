use std::marker::PhantomData;
use na::Translate;
use na;
use math::{Scalar, Point, Vect};
use entities::shape::Ball;
use entities::inspection::Repr;
use queries::geometry::Contact;
use queries::geometry::contacts_internal;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


/// Collision detector between two balls.
pub struct BallBall<N, P, V, M> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>,
    mat_type:   PhantomData<M> // FIXME: can we avoid this (using a generalized where clause ?)
}

impl<N: Clone, P: Clone, V: Clone, M> Clone for BallBall<N, P, V, M> {
    fn clone(&self) -> BallBall<N, P, V, M> {
        BallBall {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone(),
            mat_type:   PhantomData
        }
    }
}

impl<N, P, V, M> BallBall<N, P, V, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(prediction: N) -> BallBall<N, P, V, M> {
        BallBall {
            prediction: prediction,
            contact:    None,
            mat_type:   PhantomData
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M> for BallBall<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> {
    fn update(&mut self,
              _:  &CollisionDispatcher<N, P, V, M>,
              ma: &M,
              a:  &Repr<N, P, V, M>,
              mb: &M,
              b:  &Repr<N, P, V, M>)
              -> bool {
        let ra = a.repr();
        let rb = b.repr();

        if let (Some(a), Some(b)) = (ra.downcast_ref::<Ball<N>>(), rb.downcast_ref::<Ball<N>>()) {
            self.contact = contacts_internal::ball_against_ball(
                &ma.translate(&na::orig()),
                a,
                &mb.translate(&na::orig()),
                b,
                self.prediction);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_colls(&self) -> usize {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}
