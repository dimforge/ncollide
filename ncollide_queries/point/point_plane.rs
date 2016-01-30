use num::Float;
use na::Transform;
use na;
use point::PointQuery;
use entities::shape::Plane;
use math::{Point, Vect};

impl<P, M> PointQuery<P, M> for Plane<P::Vect>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let ls_pt = m.inv_transform(pt);
        let d     = na::dot(self.normal(), ls_pt.as_vec());

        if d < na::zero() && solid {
            pt.clone()
        }
        else {
            *pt + (-*self.normal() * d)
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        let ls_pt = m.inv_transform(pt);

        na::dot(self.normal(), ls_pt.as_vec()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inv_transform(pt);

        na::dot(self.normal(), ls_pt.as_vec()) <= na::zero()
    }
}
