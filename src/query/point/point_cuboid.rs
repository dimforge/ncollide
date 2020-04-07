use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Cuboid, FeatureId};
use na::RealField;

impl<N: RealField> PointQuery<N> for Cuboid<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).project_point(m, pt, solid)
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).project_point_with_feature(m, pt)
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(m, pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).contains_point(m, pt)
    }
}
