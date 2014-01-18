use std::num::Zero;
use nalgebra::na::Identity;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use bounding_volume::AABB;
use geom::Box;
use implicit::HasMargin;
use ray::{Ray, RayCast};
use ray::ray_implicit::gjk_toi_and_normal_with_ray;
use math::{N, V};

impl RayCast for Box {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        if !self.margin().is_zero() {
            gjk_toi_and_normal_with_ray(
                &Identity::new(),
                self,
                &mut JohnsonSimplex::<V>::new_w_tls(),
                ray).map(|(n, _)| n)
        }
        else {
            AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(ray)
        }
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        if self.margin().is_zero() {
            AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_with_ray(ray)
        }
        else {
            gjk_toi_and_normal_with_ray(
                &Identity::new(),
                self,
                &mut JohnsonSimplex::<V>::new_w_tls(),
                ray)
        }
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<(N, V, Option<(N, N, N)>)> {
        if self.margin().is_zero() {
            AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_and_uv_with_ray(ray)
        }
        else {
            gjk_toi_and_normal_with_ray(
                &Identity::new(),
                self,
                &mut JohnsonSimplex::<V>::new_w_tls(),
                ray).map(|(t, n)| (t, n, None))
        }
    }
}
