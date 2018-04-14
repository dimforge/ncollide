use alga::general::Id;

use query::{PointProjection, PointQuery};
use shape::{Ball, FeatureId};
use bounding_volume::BoundingSphere;
use math::{Isometry, Point};

impl<N: Real> PointQuery<P, M> for BoundingSphere<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);
        let mut proj = Ball::new(self.radius()).project_point(&Id::new(), &ls_pt, solid);

        proj.point = m.transform_point(&proj.point) + self.center().coords;

        proj
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> N {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);

        Ball::new(self.radius()).distance_to_point(&Id::new(), &ls_pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);

        Ball::new(self.radius()).contains_point(&Id::new(), &ls_pt)
    }
}
