use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::shape::Plane;
use math::{Scalar, Point, Vect};

impl<N, P, V> LocalPointQuery<N, P> for Plane<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
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
    fn distance_to_point(&self, pt: &P) -> N {
        na::dot(self.normal(), pt.as_vec()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::dot(self.normal(), pt.as_vec()) <= na::zero()
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Plane<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
