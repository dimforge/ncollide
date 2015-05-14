use na::Transform;
use point::{LocalPointQuery, PointQuery};
use entities::shape::Ball;
use entities::bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};

impl<P> LocalPointQuery<P> for BoundingSphere<P>
    where P: Point {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).project_point(&ls_pt, solid) + *self.center().as_vec()
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> <P::Vect as Vect>::Scalar {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).distance_to_point(&ls_pt)
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        let ls_pt = *pt + (-*self.center().as_vec());

        Ball::new(self.radius()).contains_point(&ls_pt)
    }
}

impl<P, M> PointQuery<P, M> for BoundingSphere<P>
    where P: Point,
          M: Transform<P> {
}
