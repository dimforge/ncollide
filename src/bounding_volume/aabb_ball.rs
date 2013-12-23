use nalgebra::na::{AlgebraicVecExt, ScalarSub, ScalarAdd, Translation};
use bounding_volume::{HasAABB, AABB};
use geom::Ball;

impl<N: Clone,
     V: AlgebraicVecExt<N> + Ord,
     M: Translation<V>>
HasAABB<N, V, M> for Ball<N> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        AABB::new(m.translation().sub_s(&self.radius()),
                  m.translation().add_s(&self.radius()))
    }
}
