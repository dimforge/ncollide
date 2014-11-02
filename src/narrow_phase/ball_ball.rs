use std::num::Zero;
use na::Translate;
use na;
use shape::Ball;
use narrow_phase::{CollisionDetector, Contact};
use ray::{Ray, ball_toi_with_ray};
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
        self.contact = collide(
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

/// Computes the contact point between two balls.
///
/// The balls must penetrate to have contact points.
#[inline]
pub fn collide<N, P, V>(center1: &P, b1: &Ball<N>, center2: &P, b2: &Ball<N>, prediction: N) -> Option<Contact<N, P, V>>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = *center2 - *center1;
    let sqdist     = na::sqnorm(&delta_pos);
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + prediction;

    if sqdist < sum_radius_with_error * sum_radius_with_error {
        let mut normal = na::normalize(&delta_pos);

        if sqdist.is_zero() {
            normal = na::canonical_basis_element(0).unwrap();
        }

        Some(Contact::new(
                *center1 + normal * r1,
                *center2 + (-normal * r2),
                normal,
                (sum_radius - sqdist.sqrt())))
    }
    else {
        None
    }
}

/// Computes the Time Of Impact of two balls.
///
/// Arguments:
/// * `m1`  - the first ball transform.
/// * `dir` - the direction of the first shape movement.
/// * `b1`  - the first ball.
/// * `m2`  - the second ball transform.
/// * `b2`  - the second ball.
#[inline]
pub fn toi<N, P, V, M>(c1: &M, dir: &V, b1: &Ball<N>, c2: &M, b2: &Ball<N>) -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> {
    // Here again, we cast a ray on the CSO exept we know that our CSO is just another bigger ball!
    let radius = b1.radius() + b2.radius();
    let center = c2.inv_translate(&c1.translate(&na::orig()));

    ball_toi_with_ray(center, radius, &Ray::new(na::orig(), -*dir), true).val1()
}
