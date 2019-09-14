use crate::common::{generate, generate_trimesh_around_origin, unref};
use na::Isometry3;
use ncollide3d::bounding_volume::{BoundingSphere, AABB};
use ncollide3d::bounding_volume::{BoundingVolume, HasBoundingVolume};
use ncollide3d::shape::{
    Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, TriMesh, Triangle,
};
use rand::SeedableRng;
use rand_isaac::IsaacRng;
use test::Bencher;

#[path = "../common/macros.rs"]
#[macro_use]
mod macros;

/*
 * Bounding volume methods.
 */
bench_method!(
    bench_aabb_intersects_aabb_always_true,
    intersects,
    aabb1: AABB<f32>,
    aabb2: AABB<f32>
);
bench_method!(
    bench_bounding_sphere_intersects_bounding_sphere_always_true,
    intersects,
    bs1: BoundingSphere<f32>,
    bs2: BoundingSphere<f32>
);

bench_method!(
    bench_aabb_contains_aabb,
    contains,
    aabb1: AABB<f32>,
    aabb2: AABB<f32>
);
bench_method!(
    bench_bounding_sphere_contains_bounding_sphere,
    contains,
    bs1: BoundingSphere<f32>,
    bs2: BoundingSphere<f32>
);

bench_method!(
    bench_aabb_merged_aabb,
    merged,
    aabb1: AABB<f32>,
    aabb2: AABB<f32>
);
bench_method!(
    bench_bounding_sphere_merged_bounding_sphere,
    merged,
    bs1: BoundingSphere<f32>,
    bs2: BoundingSphere<f32>
);

bench_method!(
    bench_aabb_loosened_aabb,
    loosened,
    aabb1: AABB<f32>,
    margin: f32
);
bench_method!(
    bench_bounding_sphere_loosened_bounding_sphere,
    loosened,
    bs1: BoundingSphere<f32>,
    margin: f32
);

/*
 * Bounding volume construction.
 */
bench_method!(
    bench_cuboid_aabb,
    bounding_volume: AABB<f32>,
    c: Cuboid<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_cuboid_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Cuboid<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_ball_aabb,
    bounding_volume: AABB<f32>,
    b: Ball<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_ball_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    b: Ball<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_capsule_aabb,
    bounding_volume: AABB<f32>,
    c: Capsule<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_capsule_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Capsule<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_cone_aabb,
    bounding_volume: AABB<f32>,
    c: Cone<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_cone_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Cone<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_cylinder_aabb,
    bounding_volume: AABB<f32>,
    c: Cylinder<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_cylinder_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Cylinder<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_segment_aabb,
    bounding_volume: AABB<f32>,
    c: Segment<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_segment_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Segment<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_triangle_aabb,
    bounding_volume: AABB<f32>,
    c: Triangle<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_triangle_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: Triangle<f32>,
    m: Isometry3<f32>
);

bench_method!(
    bench_convex_aabb,
    bounding_volume: AABB<f32>,
    c: ConvexHull<f32>,
    m: Isometry3<f32>
);
bench_method!(
    bench_convex_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    c: ConvexHull<f32>,
    m: Isometry3<f32>
);

bench_method_gen!(
    bench_mesh_aabb,
    bounding_volume: AABB<f32>,
    mesh: TriMesh<f32> = generate_trimesh_around_origin,
    m: Isometry3<f32> = generate
);
bench_method_gen!(
    bench_mesh_bounding_sphere,
    bounding_volume: BoundingSphere<f32>,
    mesh: TriMesh<f32> = generate_trimesh_around_origin,
    m: Isometry3<f32> = generate
);
