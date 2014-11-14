use na::Transform;
use na;
use bounding_volume::AABB;
use shape::Cuboid;
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};


impl<N, P, V> LocalPointQuery<N, P> for Cuboid<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let dl = na::orig::<P>() + (-*self.half_extents());
        let ur = na::orig::<P>() + *self.half_extents();
        AABB::new(dl, ur).project_point(pt, solid)
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
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

impl<N, P, V, M> PointQuery<N, P, M> for Cuboid<V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
