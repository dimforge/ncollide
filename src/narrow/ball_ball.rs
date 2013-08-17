use nalgebra::traits::norm::Norm;
use nalgebra::traits::basis::Basis;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::translation::Translation;
use geom::ball::Ball;
use narrow::collision_detector::CollisionDetector;
use contact::Contact;

/**
 * Collision detector between two balls.
 *
 * # Parameters:
 *   * `N` - type of a ball radius.
 *   * `V` - type of a ball center.
 */
pub struct BallBall<N, V, M> {
    priv contact: Option<Contact<N, V>>
}

impl<N, V, M> BallBall<N, V, M> {
    /// Creates a new persistant collision detector between two balls.
    #[inline]
    pub fn new() -> BallBall<N, V, M> {
        BallBall { contact: None }
    }
}

impl<N: Real + NumCast + Clone,
     V: VectorSpace<N> + Norm<N> + Basis + Clone,
     M: Translation<V>> 
     CollisionDetector<N, V, M, Ball<N>, Ball<N>> for
BallBall<N, V, M> {
    fn update(&mut self, ma: &M, a: &Ball<N>, mb: &M, b: &Ball<N>) {
        self.contact = collide_ball_ball(&ma.translation(), a, &mb.translation(), b);
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
}

/// Computes the contact point between two balls. The balls must penetrate to have contact points.
pub fn collide_ball_ball<V: VectorSpace<N> + Norm<N> + Basis + Clone, N: Real + NumCast + Clone>
(center1: &V, b1: &Ball<N>, center2: &V, b2: &Ball<N>) -> Option<Contact<N, V>> {
    let r1         = b1.radius();
    let r2         = b2.radius();
    let delta_pos  = center2 - *center1;
    let sqdist     = delta_pos.sqnorm();
    let sum_radius = r1 + r2;
    let sum_radius_with_error = sum_radius + NumCast::from(0.1);

    if sqdist < sum_radius_with_error * sum_radius_with_error {
        let mut normal = delta_pos.normalized();
        
        if sqdist.is_zero() {
            do Basis::canonical_basis() |b| {
                normal = b;

                false
            }
        }

        Some(Contact::new(
                center1 + normal.scalar_mul(&r1),
                center2 - normal.scalar_mul(&r2),
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
pub fn closest_points<N: Clone,
                      V: ScalarMul<N> + Sub<V, V> + Add<V, V> + Norm<N> + Clone>
(center1: &V, b1: &Ball<N>, center2: &V, b2: &Ball<N>) -> (V, V) {
    let r1     = b1.radius();
    let r2     = b2.radius();
    let normal = (center2 - *center1).normalized();

    (center1 + normal.scalar_mul(&r1), center2 - normal.scalar_mul(&r2))
}
