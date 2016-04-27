use std::marker::PhantomData;
use na::Translate;
use na;
use math::{Point, Vector, Isometry};
use geometry::shape::{Shape, Ball};
use geometry::query::Contact;
use geometry::query::contacts_internal;
use narrow_phase::{ContactGenerator, ContactDispatcher};


/// Collision detector between two balls.
pub struct BallBallContactGenerator<P: Point, M> {
    contact:  Option<Contact<P>>,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallBallContactGenerator<P, M> {
    fn clone(&self) -> BallBallContactGenerator<P, M> {
        BallBallContactGenerator {
            contact:  self.contact.clone(),
            mat_type: PhantomData
        }
    }
}

impl<P: Point, M> BallBallContactGenerator<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallContactGenerator<P, M> {
        BallBallContactGenerator {
            contact:  None,
            mat_type: PhantomData
        }
    }
}

impl<P, M> ContactGenerator<P, M> for BallBallContactGenerator<P, M>
    where P: Point,
          M: Isometry<P> {
    fn update(&mut self,
              _:          &ContactDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<<P::Vect as Vector>::Scalar>>(),
                                     b.as_shape::<Ball<<P::Vect as Vector>::Scalar>>()) {
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
    fn num_contacts(&self) -> usize {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_contacts.push(c.clone()),
            None        => ()
        }
    }
}
