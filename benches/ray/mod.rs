use rand::IsaacRng;
use test::Bencher;
use test;
use na::Id;
use ncollide::bounding_volume::{AABB3, BoundingSphere3};
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3, TriMesh3, Segment3, Triangle3,
                      Convex3};
use ncollide::ray::{RayCast, Ray3};
use common::{unref, generate, generate_trimesh_around_origin};

#[path="../common/macros.rs"]
#[macro_use] mod macros;

// FIXME: will the randomness of `solid` affect too much the benchmark?
bench_method!(bench_ray_against_ball, toi_with_ray,
              b: Ball3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cuboid, toi_with_ray,
              c: Cuboid3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_capsule, toi_with_ray,
              c: Capsule3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cone, toi_with_ray,
              c: Cone3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cylinder, toi_with_ray,
              c: Cylinder3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_aabb, toi_with_ray,
              a: AABB3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_bounding_sphere, toi_with_ray,
              b: BoundingSphere3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_ball_with_normal_uv, toi_and_normal_and_uv_with_ray,
              b: Ball3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cuboid_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cuboid3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_capsule_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Capsule3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cone_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cone3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_cylinder_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Cylinder3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_segment_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Segment3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_triangle_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Triangle3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method!(bench_ray_against_convex_with_normal_uv, toi_and_normal_and_uv_with_ray,
              c: Convex3<f32>, id: Id, ray: Ray3<f32>, solid: bool);

bench_method_gen!(bench_ray_against_trimesh_with_normal_uv, toi_and_normal_and_uv_with_ray,
                  m: TriMesh3<f32> = generate_trimesh_around_origin,
                  id: Id = generate,
                  ray: Ray3<f32> = generate,
                  solid: bool = generate);
