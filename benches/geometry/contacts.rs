use test::Bencher;
use test;
use rand::IsaacRng;
use na::Isometry3;
use ncollide::shape::{Ball3, Cuboid3, Capsule3, Cone3, Cylinder3};
use ncollide::geometry;
use common::{unref, generate};

#[path="../common/macros.rs"]
#[macro_use] mod macros;

bench_free_fn!(bench_ball_against_ball, geometry::contact,
               m1: Isometry3<f32>, b1: Ball3<f32>,
               m2: Isometry3<f32>, b2: Ball3<f32>,
               prediction: f32);

bench_free_fn!(bench_cuboid_against_cuboid, geometry::contact,
               m1: Isometry3<f32>, b1: Cuboid3<f32>,
               m2: Isometry3<f32>, b2: Cuboid3<f32>,
               prediction: f32);

bench_free_fn!(bench_capsule_against_capsule, geometry::contact,
               m1: Isometry3<f32>, b1: Capsule3<f32>,
               m2: Isometry3<f32>, b2: Capsule3<f32>,
               prediction: f32);

bench_free_fn!(bench_cone_against_cone, geometry::contact,
               m1: Isometry3<f32>, b1: Cone3<f32>,
               m2: Isometry3<f32>, b2: Cone3<f32>,
               prediction: f32);

bench_free_fn!(bench_cylinder_against_cylinder, geometry::contact,
               m1: Isometry3<f32>, b1: Cylinder3<f32>,
               m2: Isometry3<f32>, b2: Cylinder3<f32>,
               prediction: f32);
