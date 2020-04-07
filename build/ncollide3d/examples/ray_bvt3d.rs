extern crate nalgebra as na;
extern crate ncollide3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::bounding_volume::{self, BoundingSphere, HasBoundingVolume};
use ncollide3d::partitioning::{VisitStatus, BVH, BVT};
use ncollide3d::query::{visitors::RayInterferencesVisitor, Ray, RayCast};
use ncollide3d::shape::{Ball, Capsule, Cone, Cuboid};

/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape3: HasBoundingVolume<f64, BoundingSphere<f64>> + RayCast<f64> {}

impl<T> Shape3 for T where T: HasBoundingVolume<f64, BoundingSphere<f64>> + RayCast<f64> {}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vector3::new(1.0, 0.5, 1.0));

    let shapes = [
        &ball as &dyn Shape3,
        &caps as &dyn Shape3,
        &cone as &dyn Shape3,
        &cube as &dyn Shape3,
    ];

    let poss = [
        Isometry3::new(Vector3::new(0.0, 0.0, 1.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, 2.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 0.0, 3.0), na::zero()),
        Isometry3::new(Vector3::new(0.0, 2.0, 4.0), na::zero()),
    ];

    let idx_and_bounding_spheres: Vec<(usize, BoundingSphere<f64>)> = vec![
        (
            0usize,
            bounding_volume::bounding_sphere(shapes[0], &poss[0]),
        ),
        (
            1usize,
            bounding_volume::bounding_sphere(shapes[1], &poss[1]),
        ),
        (
            2usize,
            bounding_volume::bounding_sphere(shapes[2], &poss[2]),
        ),
        (
            3usize,
            bounding_volume::bounding_sphere(shapes[3], &poss[3]),
        ),
    ];

    let bvt = BVT::new_balanced(idx_and_bounding_spheres);
    let ray_hit = Ray::new(Point3::origin(), Vector3::z());
    let ray_miss = Ray::new(Point3::origin(), -Vector3::z());

    /*
     * Ray cast using a visitor.
     */
    let mut hit_count = 0;
    let mut miss_count = 0;

    // We need a new scope here to avoid borrowing issues.
    {
        let mut visitor_hit = RayInterferencesVisitor::new(&ray_hit, std::f64::MAX, |_| {
            hit_count += 1;
            VisitStatus::Continue
        });
        let mut visitor_miss = RayInterferencesVisitor::new(&ray_miss, std::f64::MAX, |_| {
            miss_count += 1;
            VisitStatus::Continue
        });

        bvt.visit(&mut visitor_hit);
        bvt.visit(&mut visitor_miss);
    }

    assert_eq!(hit_count, 3);
    assert_eq!(miss_count, 0);
}
