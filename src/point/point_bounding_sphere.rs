use na::Transform;
use point::{LocalPointQuery, PointQuery};
use shape::Ball;
use bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};

impl<N, P, V> LocalPointQuery<N, P> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).project_point(&ls_pt, solid) + *self.center().as_vec()
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).distance_to_point(&ls_pt)
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).contains_point(&ls_pt)
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for BoundingSphere<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
