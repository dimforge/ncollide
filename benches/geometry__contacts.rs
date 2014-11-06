extern crate test;
extern crate "nalgebra" as na;
extern crate ncollide;

use std::rand::{IsaacRng, Rng};
use test::Bencher;
use na::{Vec3, Iso3};
use ncollide::shape::{Ball, Cuboid, Capsule, Cone};
use ncollide::geometry;


#[bench]
fn bench_ball_against_ball(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Ball::new(rng.gen::<f32>().abs()));
    let b2s = Vec::from_fn(10000, |_| Ball::new(rng.gen::<f32>().abs()));
    let m1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let m2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let mut res = Vec::with_capacity(10000);

    bh.iter(|| {
        for ((b1, m1), (b2, m2)) in b1s.iter().zip(m1s.iter()).zip(b2s.iter().zip(m2s.iter())) {
            res.push(geometry::contact(m1, b1, m2, b2, 1.0));
        }

        res.clear();
    })
}

#[bench]
fn bench_ball_against_ball_using_shape_against_shape(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Ball::new(rng.gen::<f32>().abs()));
    let b2s = Vec::from_fn(10000, |_| Ball::new(rng.gen::<f32>().abs()));
    let m1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let m2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());

    bh.iter(|| {
        for ((b1, m1), (b2, m2)) in b1s.iter().zip(m1s.iter()).zip(b2s.iter().zip(m2s.iter())) {
            geometry::contacts_internal::shape_against_shape(m1, b1, m2, b2, 1.0);
        }
    })
}

#[bench]
fn bench_cuboid_against_cuboid(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Cuboid::new(na::abs(&rng.gen::<Vec3<f32>>())));
    let b2s = Vec::from_fn(10000, |_| Cuboid::new(na::abs(&rng.gen::<Vec3<f32>>())));
    let p1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let p2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());

    bh.iter(|| {
        for ((b1, p1), (b2, p2)) in b1s.iter().zip(p1s.iter()).zip(b2s.iter().zip(p2s.iter())) {
            geometry::contact(p1, b1, p2, b2, 1.0);
        }
    })
}

#[bench]
fn bench_cuboid_against_cuboid_using_shape_against_shape(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Cuboid::new(na::abs(&rng.gen::<Vec3<f32>>())));
    let b2s = Vec::from_fn(10000, |_| Cuboid::new(na::abs(&rng.gen::<Vec3<f32>>())));
    let m1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let m2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());

    bh.iter(|| {
        for ((b1, m1), (b2, m2)) in b1s.iter().zip(m1s.iter()).zip(b2s.iter().zip(m2s.iter())) {
            geometry::contacts_internal::shape_against_shape(m1, b1, m2, b2, 1.0);
        }
    })
}

#[bench]
fn bench_capsule_against_capsule(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Capsule::new(rng.gen::<f32>().abs(), rng.gen::<f32>().abs()));
    let b2s = Vec::from_fn(10000, |_| Capsule::new(rng.gen::<f32>().abs(), rng.gen::<f32>().abs()));
    let p1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let p2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let mut res = Vec::with_capacity(10000);

    bh.iter(|| {
        for ((b1, p1), (b2, p2)) in b1s.iter().zip(p1s.iter()).zip(b2s.iter().zip(p2s.iter())) {
            res.push(geometry::contact(p1, b1, p2, b2, 1.0));
        }

        res.clear();
    })
}

#[bench]
fn bench_cone_against_cone(bh: &mut Bencher) {
    let mut rng = IsaacRng::new_unseeded();

    let b1s = Vec::from_fn(10000, |_| Cone::new(rng.gen::<f32>().abs(), rng.gen::<f32>().abs()));
    let b2s = Vec::from_fn(10000, |_| Cone::new(rng.gen::<f32>().abs(), rng.gen::<f32>().abs()));
    let p1s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let p2s = Vec::from_fn(10000, |_| rng.gen::<Iso3<f32>>());
    let mut res = Vec::with_capacity(10000);

    bh.iter(|| {
        for ((b1, p1), (b2, p2)) in b1s.iter().zip(p1s.iter()).zip(b2s.iter().zip(p2s.iter())) {
            res.push(geometry::contact(p1, b1, p2, b2, 1.0));
        }

        res.clear();
    })
}
