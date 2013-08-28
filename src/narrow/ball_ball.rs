use std::num::Zero;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::vector::{AlgebraicVec, AlgebraicVecExt};
use geom::ball::Ball;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;
use ray::ray::{Ray, RayCastWithTransform};

/**
 * Collision detector between two balls.
 *
 * # Parameters:
 *   * `N` - type of a ball radius.
 *   * `V` - type of a ball center.
 */
pub struct BallBall<N, V, M> {
    priv prediction: N,
    priv contact:    Option<Contact<N, V>>
}

impl<N, V, M> BallBall<N, V, M> {
    /// Creates a new persistant collision detector between two balls.
    #[inline]
    pub fn new(prediction: N) -> BallBall<N, V, M> {
        BallBall {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N: Real + NumCast + Clone,
     V: AlgebraicVecExt<N> + Translation<V> + Rotate<V> + Transform<V> + Clone,
     M: Translation<V>> 
     CollisionDetector<N, V, M, Ball<N>, Ball<N>> for
BallBall<N, V, M> {
    fn update(&mut self, ma: &M, a: &Ball<N>, mb: &M, b: &Ball<N>) {
        self.contact = collide(
            &ma.translation(),
            a,
            &mb.translation(),
            b,
            &self.prediction);
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }

    #[inline]
    fn toi(_: Option<BallBall<N, V, M>>, c1: &M, dir: &V, _: &N, b1: &Ball<N>, c2: &M, b2: &Ball<N>) -> Option<N> {
        toi(c1, dir, b1, c2, b2)
    }
}

/// Computes the contact point between two balls. The balls must penetrate to have contact points.
#[inline]
pub fn collide<V: AlgebraicVecExt<N> + Clone, N: Real + NumCast + Clone>
(center1: &V, b1: &Ball<N>, center2: &V, b2: &Ball<N>, prediction: &N) -> Option<Contact<N, V>> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = center2 - *center1;
    let sqdist     = delta_pos.sqnorm();
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + *prediction;

    if sqdist < sum_radius_with_error * sum_radius_with_error {
        let mut normal = delta_pos.normalized();
        
        if sqdist.is_zero() {
            do Basis::canonical_basis() |b| {
                normal = b;

                false
            }
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

/// Computes the cloest points between two balls. If they are intersecting, the points
/// corresponding to the penetration depth are returned.
#[inline]
pub fn closest_points<N: Algebraic + Clone,
                      V: AlgebraicVec<N> + Clone>
(center1: &V, b1: &Ball<N>, center2: &V, b2: &Ball<N>) -> (V, V) {
    let r1     = b1.radius();
    let r2     = b2.radius();
    let normal = (center2 - *center1).normalized();

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
pub fn toi<N: Num + Algebraic + Ord + Clone,
           V: AlgebraicVec<N> + Translation<V> + Rotate<V> + Transform<V> + Clone,
           M: Translation<V>>(
           c1:  &M,
           dir: &V,
           b1:  &Ball<N>,
           c2:  &M,
           b2:  &Ball<N>) -> Option<N> {
    // Here again, we cast a ray on the CSO exept we know that our CSO is just another bigger ball!
    let cso = Ball::new(b1.radius() + b2.radius());

    cso.toi_with_transform_and_ray(
        &(c1.translation() - c2.translation()),
        &Ray::new(Zero::zero(), -dir)
    )
}
