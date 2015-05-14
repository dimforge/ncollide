use num::Float;
use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

impl<P> LocalPointQuery<P> for Plane<P::Vect>
    where P: Point {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let d = na::dot(self.normal(), pt.as_vec());

        if d < na::zero() && solid {
            pt.clone()
        }
        else {
            *pt + (-*self.normal() * d)
        }
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dot(self.normal(), pt.as_vec()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::dot(self.normal(), pt.as_vec()) <= na::zero()
    }
}

impl<P, M> PointQuery<P, M> for Plane<P::Vect>
    where P: Point,
          M: Transform<P> {
}
