use na::{Transform, Rotate};
use na;
use entities::bounding_volume::AABB;
use entities::shape::Cuboid;
use point::PointQuery;
use math::{Point, Vect};

impl<P, M> PointQuery<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).project_point(m, pt, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(m, pt)
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).contains_point(m, pt)
    }
}
