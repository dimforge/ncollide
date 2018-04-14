use bounding_volume::AABB;
use shape::{Cuboid, FeatureId};
use query::{PointProjection, PointQuery};
use math::{Isometry, Point};

impl<N: Real> PointQuery<P, M> for Cuboid<Vector<N>> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> PointProjection<P> {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).project_point(m, pt, solid)
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &P) -> (PointProjection<P>, FeatureId) {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).project_point_with_feature(m, pt)
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> N {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(m, pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &P) -> bool {
        let dl = Point::origin() + (-*self.half_extents());
        let ur = Point::origin() + *self.half_extents();
        AABB::new(dl, ur).contains_point(m, pt)
    }
}
