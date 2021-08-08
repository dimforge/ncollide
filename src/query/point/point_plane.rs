use crate::math::{Isometry, Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{FeatureId, Plane};
use na::{self, RealField};

impl<N: RealField + Copy> PointQuery<N> for Plane<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(pt);
        let d = self.normal.dot(&ls_pt.coords);

        let inside = d <= na::zero();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            PointProjection::new(inside, *pt + (-*self.normal * d))
        }
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let ls_pt = m.inverse_transform_point(pt);
        let dist = self.normal.dot(&ls_pt.coords);

        if dist < na::zero() && solid {
            na::zero()
        } else {
            // This will automatically be negative if the point is inside.
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(pt);

        self.normal.dot(&ls_pt.coords) <= na::zero()
    }
}
