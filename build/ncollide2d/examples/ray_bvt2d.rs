extern crate nalgebra as na;

use na::{Isometry2, Point2, Vector2};
use ncollide2d::bounding_volume::{self, BoundingSphere, HasBoundingVolume};
use ncollide2d::partitioning::{VisitStatus, BVH, BVT};
use ncollide2d::query::{visitors::RayInterferencesVisitor, Ray, RayCast};
use ncollide2d::shape::{Ball, Cuboid};

/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape: HasBoundingVolume<f64, BoundingSphere<f64>> + RayCast<f64> {}

impl<T> Shape for T where T: HasBoundingVolume<f64, BoundingSphere<f64>> + RayCast<f64> {}

fn main() {
    let ball1 = Ball::new(0.5);
    let ball2 = Ball::new(0.75);
    let cube1 = Cuboid::new(Vector2::new(0.5, 0.75));
    let cube2 = Cuboid::new(Vector2::new(1.0, 0.5));

    let shapes = [
        &ball1 as &dyn Shape,
        &ball2 as &dyn Shape,
        &cube1 as &dyn Shape,
        &cube2 as &dyn Shape,
    ];

    let poss = [
        Isometry2::new(Vector2::new(1.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(2.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(3.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(4.0, 2.0), na::zero()),
    ];

    // FIXME: why do we need the explicit type annotation here?
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
    let ray_hit = Ray::new(Point2::origin(), Vector2::x());
    let ray_miss = Ray::new(Point2::origin(), -Vector2::x());

    /*
     * Collecting all objects with bounding volumes intersecting the ray.
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
