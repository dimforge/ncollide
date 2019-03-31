use crate::bounding_volume::BoundingSphere;
use crate::math::{Isometry, Point};
use na::RealField;
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Ball, FeatureId};

impl<N: RealField> PointQuery<N> for BoundingSphere<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);
        let mut proj = Ball::new(self.radius()).project_point(&Isometry::identity(), &ls_pt, solid);

        proj.point = m * proj.point + self.center().coords;

        proj
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);

        Ball::new(self.radius()).distance_to_point(&Isometry::identity(), &ls_pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coords);

        Ball::new(self.radius()).contains_point(&Isometry::identity(), &ls_pt)
    }
}
