use common::{generate, unref};
use na::Isometry3;
use ncollide3d::query;
use ncollide3d::shape::{Ball, Cuboid};
use rand::IsaacRng;
use test;
use test::Bencher;

#[path = "../common/macros.rs"]
#[macro_use]
mod macros;

bench_free_fn!(
    bench_ball_against_ball,
    query::contact,
    m1: Isometry3<f32>,
    b1: Ball<f32>,
    m2: Isometry3<f32>,
    b2: Ball<f32>,
    prediction: f32
);

bench_free_fn!(
    bench_cuboid_against_cuboid,
    query::contact,
    m1: Isometry3<f32>,
    b1: Cuboid<f32>,
    m2: Isometry3<f32>,
    b2: Cuboid<f32>,
    prediction: f32
);

//bench_free_fn!(
//    bench_capsule_against_capsule,
//    query::contact,
//    m1: Isometry3<f32>,
//    b1: Capsule<f32>,
//    m2: Isometry3<f32>,
//    b2: Capsule<f32>,
//    prediction: f32
//);
//
//bench_free_fn!(
//    bench_cone_against_cone,
//    query::contact,
//    m1: Isometry3<f32>,
//    b1: Cone<f32>,
//    m2: Isometry3<f32>,
//    b2: Cone<f32>,
//    prediction: f32
//);
//
//bench_free_fn!(
//    bench_cylinder_against_cylinder,
//    query::contact,
//    m1: Isometry3<f32>,
//    b1: Cylinder<f32>,
//    m2: Isometry3<f32>,
//    b2: Cylinder<f32>,
//    prediction: f32
//);
