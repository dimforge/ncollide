use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::vector::VecExt;
use bounding_volume::aabb::AABB;
use geom::box::Box;
use ray::ray::{Ray, RayCast};

impl<N: Primitive + Orderable + ToStr,
     V: VecExt<N> + Clone + ToStr,
     M: Rotate<V> + Transform<V>>
RayCast<N, V, M> for Box<N, V> {
    #[inline]
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(&ls_ray)
    }
}
