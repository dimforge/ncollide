use std::num::Signed;
use nalgebra::na::{AlgebraicVecExt, AbsoluteRotate, Translation};
use bounding_volume::{HasAABB, AABB};
use geom::Box;

impl<N: Algebraic + Signed + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: AbsoluteRotate<V> + Translation<V>>
HasAABB<N, V, M> for Box<N, V> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        let center          = m.translation();
        let ws_half_extents = m.absolute_rotate(&self.half_extents().add_s(&self.margin()));

        AABB::new(center - ws_half_extents, center + ws_half_extents)
    }
}
