use na::Transform;
use na;
use entities::bounding_volume::AABB;
use entities::shape::Cuboid;
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};


impl<P> LocalPointQuery<P> for Cuboid<P::Vect>
    where P: Point {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).project_point(pt, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> <P::Vect as Vect>::Scalar {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).distance_to_point(pt)
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).contains_point(pt)
    }
}

impl<P, M> PointQuery<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Transform<P> {
}
