use na::Translate;
use na;
use shape::Ball;
use narrow_phase::CollisionDetector;
use geometry::Contact;
use geometry::contacts_internal;
use math::{Scalar, Point, Vect};


/// Collision detector between two balls.
#[deriving(Encodable, Decodable)]
pub struct BallBall<N, P, V, M> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N: Clone, P: Clone, V: Clone, M> Clone for BallBall<N, P, V, M> {
    fn clone(&self) -> BallBall<N, P, V, M> {
        BallBall {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<N, P, V, M> BallBall<N, P, V, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(prediction: N) -> BallBall<N, P, V, M> {
        BallBall {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M, Ball<N>, Ball<N>> for BallBall<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> {
    fn update(&mut self, ma: &M, a: &Ball<N>, mb: &M, b: &Ball<N>) {
        self.contact = contacts_internal::ball_against_ball(
            &ma.translate(&na::orig()),
            a,
            &mb.translate(&na::orig()),
            b,
            self.prediction);
    }

    #[inline]
    fn num_colls(&self) -> uint {
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
