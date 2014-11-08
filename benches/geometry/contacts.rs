use std::rand::IsaacRng;
use test::Bencher;
use test;
use na::Iso3;
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3};
use ncollide::geometry;
use common::{unref, generate};

#[path="../common/macros.rs"]
mod macros;

bench_free_fn!(bench_ball_against_ball, geometry::contact,
               m1: Iso3<f32>, b1: Ball3,
               m2: Iso3<f32>, b2: Ball3,
               prediction: f32)

bench_free_fn!(bench_cuboid_against_cuboid, geometry::contact,
               m1: Iso3<f32>, b1: Cuboid3,
               m2: Iso3<f32>, b2: Cuboid3,
               prediction: f32)

bench_free_fn!(bench_capsule_against_capsule, geometry::contact,
               m1: Iso3<f32>, b1: Capsule3,
               m2: Iso3<f32>, b2: Capsule3,
               prediction: f32)

bench_free_fn!(bench_cone_against_cone, geometry::contact,
               m1: Iso3<f32>, b1: Cone3,
               m2: Iso3<f32>, b2: Cone3,
               prediction: f32)

bench_free_fn!(bench_cylinder_against_cylinder, geometry::contact,
               m1: Iso3<f32>, b1: Cylinder3,
               m2: Iso3<f32>, b2: Cylinder3,
               prediction: f32)
