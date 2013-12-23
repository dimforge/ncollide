use nalgebra::na::{AlgebraicVecExt, Rotate, Transform};
use bounding_volume::{AABB, HasAABB};
use bounding_volume;
use geom::Cylinder;

impl<N: Signed + Algebraic + Orderable + Primitive + Clone,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Cylinder<N> {
    #[inline]
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
