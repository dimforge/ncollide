use num::Float;
use na::Transform;
use na;
use point::PointQuery;
use entities::shape::Plane;
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for Plane<P::Vect>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let ls_pt = m.inverse_transform(pt);
        let d     = na::dot(self.normal(), ls_pt.as_vector());

        if d < na::zero() && solid {
            pt.clone()
        }
        else {
            *pt + (-*self.normal() * d)
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vector>::Scalar {
        let ls_pt = m.inverse_transform(pt);

        na::dot(self.normal(), ls_pt.as_vector()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform(pt);

        na::dot(self.normal(), ls_pt.as_vector()) <= na::zero()
    }
}
