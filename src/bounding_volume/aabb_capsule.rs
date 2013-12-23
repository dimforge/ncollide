use nalgebra::na::{AlgebraicVecExt, Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Capsule;

impl<N: Algebraic + Signed + Primitive + Orderable + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Capsule<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
