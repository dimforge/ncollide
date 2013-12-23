use nalgebra::na::{AlgebraicVecExt, Rotate, Transform};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::Cone;


impl<N: Signed + Algebraic + Primitive + Orderable + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cone<N> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
