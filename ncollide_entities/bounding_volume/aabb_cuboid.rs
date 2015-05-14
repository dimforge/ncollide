use na::{AbsoluteRotate, Translate};
use na;
use bounding_volume::{HasAABB, AABB};
use shape::Cuboid;
use math::Point;

impl<P, M> HasAABB<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Translate<P> + AbsoluteRotate<P::Vect> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<P> {
        let center          = m.translate(&na::orig());
        let ws_half_extents = m.absolute_rotate(self.half_extents());

        AABB::new(center + -ws_half_extents.clone(), center + ws_half_extents)
    }
}
