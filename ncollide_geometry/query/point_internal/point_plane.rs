use na::{self, Transform};
use query::{PointQuery, PointProjection};
use shape::Plane;
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for Plane<P::Vect>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_transform(pt);
        let d     = na::dot(self.normal(), ls_pt.as_vector());

        let inside = d <= na::zero();

        if inside && solid {
            PointProjection::new(true, pt.clone())
        }
        else {
            PointProjection::new(inside, *pt + (-*self.normal() * d))
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        let ls_pt = m.inverse_transform(pt);
        let dist  = na::dot(self.normal(), ls_pt.as_vector());

        if dist < na::zero() && solid {
            na::zero()
        }
        else {
            // This will automatically be negative if the point is inside.
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform(pt);

        na::dot(self.normal(), ls_pt.as_vector()) <= na::zero()
    }
}
