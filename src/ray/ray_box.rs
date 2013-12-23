use nalgebra::na::{AlgebraicVecExt, Rotate, Transform, Identity, Cast};
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use bounding_volume::AABB;
use geom::Box;
use implicit::HasMargin;
use ray::{Ray, RayCast, RayCastWithTransform};
use ray::ray_implicit::gjk_toi_and_normal_with_ray;

impl<N: Primitive + Orderable + Algebraic + Float + Cast<f32>,
     V: AlgebraicVecExt<N> + Clone>
RayCast<N, V> for Box<N, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        if !self.margin().is_zero() {
            gjk_toi_and_normal_with_ray(
                &Identity::new(),
                self,
                &mut JohnsonSimplex::<N, V>::new_w_tls(),
                ray).map(|(n, _)| n)
        }
        else {
            AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(ray)
        }
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        if self.margin().is_zero() {
            AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_with_ray(ray)
        }
        else {
            gjk_toi_and_normal_with_ray(
                &Identity::new(),
                self,
                &mut JohnsonSimplex::<N, V>::new_w_tls(),
                ray)
        }
    }
}

impl<N: Primitive + Orderable + Algebraic + Float + Cast<f32>,
     V: AlgebraicVecExt<N> + Clone,
     M: Rotate<V> + Transform<V>>
RayCastWithTransform<N, V, M> for Box<N, V> { }
