extern crate nalgebra as na;
extern crate ncollide;

use na::{Pnt2, Vec2, Iso2};
use ncollide::partitioning::BVT;
use ncollide::shape::{Cone, Ball, Cuboid, Capsule};
use ncollide::ray::{RayInterferencesCollector, Ray, RayCast};
use ncollide::bounding_volume::HasBoundingSphere;

/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape2: HasBoundingSphere<Pnt2<f64>, Iso2<f64>> +
              RayCast<Pnt2<f64>, Iso2<f64>> {
}

impl<T> Shape2 for T
    where T: HasBoundingSphere<Pnt2<f64>, Iso2<f64>> +
             RayCast<Pnt2<f64>, Iso2<f64>> {
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vec2::new(1.0, 0.5));

    let shapes = [
        &ball as &Shape2,
        &caps as &Shape2,
        &cone as &Shape2,
        &cube as &Shape2
    ];

    let poss = [
        Iso2::new(Vec2::new(1.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(2.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(3.0, 0.0), na::zero()),
        Iso2::new(Vec2::new(4.0, 2.0), na::zero())
    ];

    let idx_and_bounding_spheres  = vec!(
        (0usize, shapes[0].bounding_sphere(&poss[0])),
        (1usize, shapes[1].bounding_sphere(&poss[1])),
        (2usize, shapes[2].bounding_sphere(&poss[2])),
        (3usize, shapes[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vec2::x());
    let ray_miss = Ray::new(na::orig(), -Vec2::x());

    /*
     * Collecting all objects with bounding volumes intersecting the ray.
     */
    let mut collector_hit:  Vec<usize> = Vec::new();
    let mut collector_miss: Vec<usize> = Vec::new();

    // We need a new scope here to avoid borrowing issues.
    {
        let mut visitor_hit  = RayInterferencesCollector::new(&ray_hit, &mut collector_hit);
        let mut visitor_miss = RayInterferencesCollector::new(&ray_miss, &mut collector_miss);

        bvt.visit(&mut visitor_hit);
        bvt.visit(&mut visitor_miss);
    }

    assert!(collector_hit.len()  == 3);
    assert!(collector_miss.len() == 0);
}
