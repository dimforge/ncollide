use std::num::{Zero, One, Bounded};
use nalgebra::na::{Transform, Rotate, AlgebraicVecExt};
use bounding_volume::{AABB, HasAABB, implicit_shape_aabb};
use geom::Convex;

impl<N: Algebraic + Ord + Bounded + Neg<N> + Clone + One + Zero + Primitive,
     V: AlgebraicVecExt<N> + Clone,
     M: Transform<V> + Rotate<V>>
HasAABB<N, V, M> for Convex<N, V> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        // XXX: optimize that
        implicit_shape_aabb(m, self)
    }
}
