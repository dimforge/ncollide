extern crate nalgebra as na;
extern crate ncollide;

use na::{Isometry2, Point2, Vector2};
use ncollide::partitioning::BVT;
use ncollide::shape::{Ball, Capsule, Cone, Cuboid};
use ncollide::query::{Ray, RayCast, RayInterferencesCollector};
use ncollide::bounding_volume::{self, BoundingSphere, HasBoundingVolume};

/*
 * Custom trait to group `HasBoudingSphere` and `RayCast` together.
 */
trait Shape2
    : HasBoundingVolume<Isometry2<f64>, BoundingSphere<Point2<f64>>>
    + RayCast<Point2<f64>, Isometry2<f64>> {
}

impl<T> Shape2 for T
where
    T: HasBoundingVolume<Isometry2<f64>, BoundingSphere<Point2<f64>>>
        + RayCast<Point2<f64>, Isometry2<f64>>,
{
}

fn main() {
    let ball = Ball::new(0.5);
    let caps = Capsule::new(0.5, 0.75);
    let cone = Cone::new(0.5, 0.75);
    let cube = Cuboid::new(Vector2::new(1.0, 0.5));

    let shapes = [
        &ball as &Shape2,
        &caps as &Shape2,
        &cone as &Shape2,
        &cube as &Shape2,
    ];

    let poss = [
        Isometry2::new(Vector2::new(1.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(2.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(3.0, 0.0), na::zero()),
        Isometry2::new(Vector2::new(4.0, 2.0), na::zero()),
    ];

    // FIXME: why do we need the explicit type annotation here?
    let idx_and_bounding_spheres: Vec<(usize, BoundingSphere<Point2<f64>>)> = vec![
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
    let ray_hit = Ray::new(na::origin(), Vector2::x());
    let ray_miss = Ray::new(na::origin(), -Vector2::x());

    /*
     * Collecting all objects with bounding volumes intersecting the ray.
     */
    let mut collector_hit: Vec<usize> = Vec::new();
    let mut collector_miss: Vec<usize> = Vec::new();

    // We need a new scope here to avoid borrowing issues.
    {
        let mut visitor_hit = RayInterferencesCollector::new(&ray_hit, &mut collector_hit);
        let mut visitor_miss = RayInterferencesCollector::new(&ray_miss, &mut collector_miss);

        bvt.visit(&mut visitor_hit);
        bvt.visit(&mut visitor_miss);
    }

    assert!(collector_hit.len() == 3);
    assert!(collector_miss.len() == 0);
}
