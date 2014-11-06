extern crate "nalgebra" as na;
extern crate ncollide;

use na::{Iso2, Vec2};
use ncollide::partitioning::{BVT, RayInterferencesCollector};
use ncollide::shape::{Shape2, Cone, Ball, Cuboid, Capsule};
use ncollide::ray::{Ray, Ray2};
use ncollide::bounding_volume::HasBoundingSphere;

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vec2::new(1.0, 0.5));

    let geoms = [
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
        (0u, geoms[0].bounding_sphere(&poss[0])),
        (1u, geoms[1].bounding_sphere(&poss[1])),
        (2u, geoms[2].bounding_sphere(&poss[2])),
        (3u, geoms[3].bounding_sphere(&poss[3]))
    );

    let bvt      = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit  = Ray::new(na::orig(), Vec2::x());
    let ray_miss = Ray::new(na::orig(), -Vec2::x());

    /*
     * Ray cast using a callback.
     */
    let mut cast_fn = |id: &uint, ray: &Ray2| {
        geoms[*id].toi_with_transform_and_ray(&poss[*id], ray, true).map(|toi| (toi, toi))
    };

    assert!(bvt.cast_ray(&ray_hit, &mut cast_fn).is_some());
    assert!(bvt.cast_ray(&ray_miss, &mut cast_fn).is_none());

    /*
     * Ray cast using a visitor.
     */
    let mut collector_hit:  Vec<uint> = Vec::new();
    let mut collector_miss: Vec<uint> = Vec::new();

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

