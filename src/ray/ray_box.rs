use nalgebra::na::{AlgebraicVecExt, Rotate, Transform};
use bounding_volume::AABB;
use geom::Box;
use ray::{Ray, RayCast, RayCastWithTransform};

impl<N: Primitive + Orderable + Algebraic,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Box<N, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(ray)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_with_ray(ray)
    }
}

impl<N: Primitive + Orderable + Algebraic,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Box<N, V> { }
