use nalgebra::vec::VecExt;
use nalgebra::mat::{Rotate, Transform};
use bounding_volume::aabb::AABB;
use geom::box::Box;
use ray::ray::{Ray, RayCast, RayCastWithTransform};

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + Clone + ToStr>
RayCast<N, V> for Box<N, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(ray)
    }
}

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Box<N, V>;
