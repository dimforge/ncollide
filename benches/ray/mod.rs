use std::rand::IsaacRng;
use test::Bencher;
use test;
use ncollide::bounding_volume::{AABB3, BoundingSphere3};
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3, Mesh3};
use ncollide::ray::{LocalRayCast, Ray3};
use common::{unref, random, generate_trimesh_around_origin};

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

bench_method!(bench_ray_against_aabb, toi_with_ray,
              a: AABB3, ray: Ray3, solid: bool)

bench_method!(bench_ray_against_bounding_sphere, toi_with_ray,
              b: BoundingSphere3, ray: Ray3, solid: bool)

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

bench_method_gen!(bench_ray_against_trimesh_with_normal_uv, toi_and_normal_and_uv_with_ray,
                  m: Mesh3 = generate_trimesh_around_origin,
                  ray: Ray3 = random,
                  solid: bool = random)
