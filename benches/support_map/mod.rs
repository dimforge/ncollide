use std::rand::IsaacRng;
use test::Bencher;
use test;
use na::{Iso3, Vec3};
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3, Segment3, Triangle3,
                      Convex3};
use ncollide::support_map::SupportMap;
use common::{unref, generate};


#[path="../common/macros.rs"]
mod macros;

bench_method!(bench_ball_support_map, support_point, c: Ball3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_cuboid_support_map, support_point, c: Cuboid3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_capsule_support_map, support_point, c: Capsule3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_cone_support_map, support_point, c: Cone3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_cylinder_support_map, support_point, c: Cylinder3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_segment_support_map, support_point, c: Segment3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_triangle_support_map, support_point, c: Triangle3, m: Iso3<f32>, dir: Vec3<f32>)
bench_method!(bench_convex_support_map, support_point, c: Convex3, m: Iso3<f32>, dir: Vec3<f32>)
