use na::{self, Transform, Rotate};
use bounding_volume::AABB;
use shape::Cuboid;
use point::{PointQuery, PointProjection};
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).project_point(m, pt, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(m, pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let dl = na::origin::<P>() + (-*self.half_extents());
        let ur = na::origin::<P>() + *self.half_extents();
        AABB::new(dl, ur).contains_point(m, pt)
    }
}
