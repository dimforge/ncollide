use nalgebra::na::AlgebraicVecExt;
use bounding_volume::{HasAABB, AABB};
use geom::Plane;

impl<V: AlgebraicVecExt<N>, N, M>
HasAABB<N, V, M> for Plane<N, V> {
    fn aabb(&self, _: &M) -> AABB<N, V> {
        let _m: V = Bounded::max_value();

        AABB::new(-_m, Bounded::max_value())
    }
}
