use na::{Identity, Transform};
use point::PointQuery;
use entities::shape::Ball;
use entities::bounding_volume::BoundingSphere;
use math::{Scalar, Point, Vect};

impl<P, M> PointQuery<P, M> for BoundingSphere<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let ls_pt = m.inv_transform(pt) + (-*self.center().as_vec());
        let proj  = Ball::new(self.radius()).project_point(&Identity::new(), &ls_pt, solid);

        m.transform(&proj) + *self.center().as_vec()
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        let ls_pt = m.inv_transform(pt) + (-*self.center().as_vec());

        Ball::new(self.radius()).distance_to_point(&Identity::new(), &ls_pt)
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inv_transform(pt) + (-*self.center().as_vec());

        Ball::new(self.radius()).contains_point(&Identity::new(), &ls_pt)
    }
}
