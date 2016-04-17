use na::{AbsoluteRotate, Translate};
use na;
use bounding_volume::{HasBoundingVolume, AABB};
use shape::Cuboid;
use math::Point;

impl<P, M> HasBoundingVolume<M, AABB<P>> for Cuboid<P::Vect>
    where P: Point,
          M: Translate<P> + AbsoluteRotate<P::Vect> {
    #[inline]
    fn bounding_volume(&self, m: &M) -> AABB<P> {
        let center          = m.translate(&na::origin());
        let ws_half_extents = m.absolute_rotate(self.half_extents());

        AABB::new(center + -ws_half_extents.clone(), center + ws_half_extents)
    }
}
