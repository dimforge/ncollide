use std::marker::PhantomData;
use na::Translate;
use na;
use math::{Point, Vector, Isometry};
use entities::shape::Ball;
use entities::inspection::Shape;
use queries::geometry::Contact;
use queries::geometry::contacts_internal;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


/// Collision detector between two balls.
pub struct BallBallCollisionDetector<P: Point, M> {
    contact:  Option<Contact<P>>,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallBallCollisionDetector<P, M> {
    fn clone(&self) -> BallBallCollisionDetector<P, M> {
        BallBallCollisionDetector {
            contact:  self.contact.clone(),
            mat_type: PhantomData
        }
    }
}

impl<P: Point, M> BallBallCollisionDetector<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallCollisionDetector<P, M> {
        BallBallCollisionDetector {
            contact:  None,
            mat_type: PhantomData
        }
    }
}

impl<P, M> CollisionDetector<P, M> for BallBallCollisionDetector<P, M>
    where P: Point,
          M: Isometry<P, P::Vect> {
    fn update(&mut self,
              _:          &CollisionDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        let ra = a.desc();
        let rb = b.desc();

        if let (Some(a), Some(b)) = (ra.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                     rb.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
            self.contact = contacts_internal::ball_against_ball(
                &ma.translate(&na::origin()),
                a,
                &mb.translate(&na::origin()),
                b,
                prediction);

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
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}
