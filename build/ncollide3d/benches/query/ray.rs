use crate::common::{generate, generate_trimesh_around_origin, unref};
use na::Isometry3;
use ncollide3d::bounding_volume::{BoundingSphere, AABB};
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::{
    Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, TriMesh, Triangle,
};
use rand::SeedableRng;
use rand_isaac::IsaacRng;
use test::Bencher;

#[path = "../common/macros.rs"]
#[macro_use]
mod macros;

// FIXME: will the randomness of `solid` affect too much the benchmark?
bench_method!(
    bench_ray_against_ball,
    toi_with_ray,
    b: Ball<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cuboid,
    toi_with_ray,
    c: Cuboid<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_capsule,
    toi_with_ray,
    c: Capsule<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cone,
    toi_with_ray,
    c: Cone<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cylinder,
    toi_with_ray,
    c: Cylinder<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_aabb,
    toi_with_ray,
    a: AABB<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_bounding_sphere,
    toi_with_ray,
    b: BoundingSphere<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_ball_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    b: Ball<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cuboid_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Cuboid<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_capsule_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Capsule<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cone_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Cone<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_cylinder_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Cylinder<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_segment_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Segment<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_triangle_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: Triangle<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method!(
    bench_ray_against_convex_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    c: ConvexHull<f32>,
    pos: Isometry3<f32>,
    ray: Ray<f32>,
    solid: bool
);

bench_method_gen!(
    bench_ray_against_trimesh_with_normal_uv,
    toi_and_normal_and_uv_with_ray,
    m: TriMesh<f32> = generate_trimesh_around_origin,
    pos: Isometry3<f32> = generate,
    ray: Ray<f32> = generate,
    solid: bool = generate
);
