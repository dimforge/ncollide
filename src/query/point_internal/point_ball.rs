use alga::general::Real;
use na;

use query::{PointProjection, PointQuery};
use shape::{Ball, FeatureId};
use math::{Isometry, Point};

impl<N: Real> PointQuery<P, M> for Ball<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_translate_point(pt);
        let distance_squared = na::norm_squared(&ls_pt.coords);

        let inside = distance_squared <= self.radius() * self.radius();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            let ls_proj = Point::origin() + ls_pt.coords / distance_squared.sqrt();

            PointProjection::new(inside, m.translate_point(&ls_proj))
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> N {
        let dist = na::norm(&m.inverse_translate_point(pt).coords) - self.radius();

        if solid && dist < na::zero() {
            na::zero()
        } else {
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &P) -> bool {
        na::norm_squared(&m.inverse_translate_point(pt).coords)
            <= self.radius() * self.radius()
    }
}
