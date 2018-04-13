use na;
use query::{PointProjection, PointQuery};
use shape::{FeatureId, Plane};
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Plane<P::Vector> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_transform_point(pt);
        let d = na::dot(self.normal().as_ref(), &ls_pt.coordinates());

        let inside = d <= na::zero();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            PointProjection::new(inside, *pt + (-*self.normal().as_ref() * d))
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, pt: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let ls_pt = m.inverse_transform_point(pt);
        let dist = na::dot(self.normal().as_ref(), &ls_pt.coordinates());

        if dist < na::zero() && solid {
            na::zero()
        } else {
            // This will automatically be negative if the point is inside.
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt);

        na::dot(self.normal().as_ref(), &ls_pt.coordinates()) <= na::zero()
    }
}
