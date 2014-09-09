use std::num::Zero;
use nalgebra::na::Identity;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use bounding_volume::AABB;
use geom::Cuboid;
use ray::{Ray, RayCast, RayIntersection};
use ray::ray_implicit::implicit_toi_and_normal_with_ray;
use math::{Scalar, Vect};

impl RayCast for Cuboid {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<Scalar> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_with_ray(ray, solid)
    }

    // #[dim3]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        AABB::new(-self.half_extents(), self.half_extents()).toi_and_normal_and_uv_with_ray(ray, solid)
    }
}
