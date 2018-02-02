use alga::general::Id;

use query::{PointProjection, PointQuery};
use shape::Ball;
use bounding_volume::BoundingSphere;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for BoundingSphere<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coordinates());
        let mut proj = Ball::new(self.radius()).project_point(&Id::new(), &ls_pt, solid);

        proj.point = m.transform_point(&proj.point) + self.center().coordinates();

        proj
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coordinates());

        Ball::new(self.radius()).distance_to_point(&Id::new(), &ls_pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt) + (-self.center().coordinates());

        Ball::new(self.radius()).contains_point(&Id::new(), &ls_pt)
    }
}
