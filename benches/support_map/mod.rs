use rand::IsaacRng;
use test::Bencher;
use test;
use na::{Isometry3, Vector3};
use ncollide::shape::{Ball3, Capsule3, Cone3, Convex3, Cuboid3, Cylinder3, Segment3, Triangle3};
use ncollide::shape::SupportMap;
use common::{generate, unref};

#[path = "../common/macros.rs"]
#[macro_use]
mod macros;

bench_method!(
    bench_ball_support_map,
    support_point,
    c: Ball3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_cuboid_support_map,
    support_point,
    c: Cuboid3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_capsule_support_map,
    support_point,
    c: Capsule3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_cone_support_map,
    support_point,
    c: Cone3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_cylinder_support_map,
    support_point,
    c: Cylinder3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_segment_support_map,
    support_point,
    c: Segment3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_triangle_support_map,
    support_point,
    c: Triangle3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
bench_method!(
    bench_convex_support_map,
    support_point,
    c: Convex3<f32>,
    m: Isometry3<f32>,
    dir: Vector3<f32>
);
