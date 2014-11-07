use std::rand::IsaacRng;
use test::Bencher;
use test;
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3};
use ncollide::ray::{LocalRayCast, Ray3};
use common::{unref, random};

#[path="../common/macros.rs"]
mod macros;

// FIXME: will the randomness of `solid` affect too much the benchmark?
bench_method!(bench_ray_against_ball, toi_with_ray,
              b: Ball3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cuboid, toi_with_ray,
              c: Cuboid3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_capsule, toi_with_ray,
              c: Capsule3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cone, toi_with_ray,
              c: Cone3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cylinder, toi_with_ray,
              c: Cylinder3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_ball_with_normal_uv, toi_and_normal_and_uv_with_ray,
              b: Ball3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cuboid_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cuboid3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_capsule_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Capsule3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cone_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cone3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_cylinder_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cylinder3, ray: Ray3, solid: bool)
