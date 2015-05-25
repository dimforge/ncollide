use num::Float;
use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::shape::Ball;
use math::{Scalar, Point, Vect};

impl<P> LocalPointQuery<P> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let sqdist = na::sqnorm(pt.as_vec());

        if sqdist <= self.radius() * self.radius() && solid {
            pt.clone()
        }
        else {
            na::orig::<P>() + *pt.as_vec() / sqdist.sqrt()
        }
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> <P::Vect as Vect>::Scalar {
        (na::norm(pt.as_vec()) - self.radius()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::sqnorm(pt.as_vec()) <= self.radius() * self.radius()
    }
}

impl<P, M> PointQuery<P, M> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> {
}
