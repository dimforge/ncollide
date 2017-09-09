use bounding_volume::AABB;
use shape::Cuboid;
use query::{PointQuery, PointProjection};
use math::{Point, Isometry};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Cuboid<P::Vector> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let dl = P::origin() + (-*self.half_extents());
        let ur = P::origin() + *self.half_extents();
        AABB::new(dl, ur).project_point(m, pt, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let dl = P::origin() + (-*self.half_extents());
        let ur = P::origin() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(m, pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let dl = P::origin() + (-*self.half_extents());
        let ur = P::origin() + *self.half_extents();
        AABB::new(dl, ur).contains_point(m, pt)
    }
}
