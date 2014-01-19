use std::num::Zero;
use nalgebra::na::Translation;
use nalgebra::na;
use geom::Ball;
use narrow::CollisionDetector;
use contact::Contact;
use ray::{Ray, ball_toi_with_ray};
use math::{N, V, M};

/// Collision detector between two balls.
#[deriving(Encodable, Decodable)]
pub struct BallBall {
    priv prediction: N,
    priv contact:    Option<Contact>
}

impl Clone for BallBall {
    fn clone(&self) -> BallBall {
        BallBall {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl BallBall {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(prediction: N) -> BallBall {
        BallBall {
            prediction: prediction,
            contact:    None
        }
    }
}

impl CollisionDetector<Ball, Ball> for
BallBall {
    fn update(&mut self, ma: &M, a: &Ball, mb: &M, b: &Ball) {
        self.contact = collide(
            &ma.translation(),
            a,
            &mb.translation(),
            b,
            &self.prediction);
    }

    #[inline]
    fn num_colls(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }

    #[inline]
    fn toi(_: Option<BallBall>, c1: &M, dir: &V, _: &N, b1: &Ball, c2: &M, b2: &Ball) -> Option<N> {
        toi(c1, dir, b1, c2, b2)
    }
}

/// Computes the contact point between two balls.
///
/// The balls must penetrate to have contact points.
#[inline]
pub fn collide(center1: &V, b1: &Ball, center2: &V, b2: &Ball, prediction: &N) -> Option<Contact> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = center2 - *center1;
    let sqdist     = na::sqnorm(&delta_pos);
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + *prediction;

    if sqdist < sum_radius_with_error * sum_radius_with_error {
        let mut normal = na::normalize(&delta_pos);
        
        if sqdist.is_zero() {
            na::canonical_basis(|b| {
                normal = b;

                false
            })
        }

        Some(Contact::new(
                center1 + normal * r1,
                center2 - normal * r2,
                normal,
                (sum_radius - sqdist.sqrt())))
    }
    else {
        None
    }
}

/// Computes the closest points between two balls.
///
/// If they are intersecting, the points corresponding to the penetration depth are returned.
#[inline]
pub fn closest_points(center1: &V, b1: &Ball, center2: &V, b2: &Ball) -> (V, V) {
    let r1     = b1.radius();
    let r2     = b2.radius();
    let normal = na::normalize(&(center2 - *center1));

    (center1 + normal * r1, center2 - normal * r2)
}

/// Computes the Time Of Impact of two balls.
///
/// Arguments:
///     * `m1`  - the first ball transform.
///     * `dir` - the direction of the first geometry movement.
///     * `b1`  - the first ball.
///     * `m2`  - the second ball transform.
///     * `b2`  - the second ball.
#[inline]
pub fn toi(c1: &M, dir: &V, b1: &Ball, c2: &M, b2: &Ball) -> Option<N> {
    // Here again, we cast a ray on the CSO exept we know that our CSO is just another bigger ball!
    let radius = b1.radius() + b2.radius();
    let center = c1.translation() - c2.translation();

    ball_toi_with_ray(center, radius, &Ray::new(na::zero(), -dir))
}
