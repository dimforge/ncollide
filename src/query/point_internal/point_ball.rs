use alga::general::Real;
use math::{Isometry, Point};
use na;
use query::{PointProjection, PointQuery};
use shape::{Ball, FeatureId};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for Ball<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(pt);
        let distance_squared = na::norm_squared(&ls_pt.coords);

        let inside = distance_squared <= self.radius() * self.radius();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            let normal = ls_pt.coords / distance_squared.sqrt();
            let proj = m * Point::from_coordinates(normal * self.radius());

            PointProjection::new(inside, proj)
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &Point<N>) -> (PointProjection<N>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let dist = na::norm(&m.inverse_transform_point(pt).coords) - self.radius();

        if solid && dist < na::zero() {
            na::zero()
        } else {
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        na::norm_squared(&m.inverse_transform_point(pt).coords)
            <= self.radius() * self.radius()
    }
}
