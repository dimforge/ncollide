extern crate nalgebra as na;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::query::{Ray, RayCast};
use ncollide3d::shape::Cuboid;

fn main() {
    let cuboid = Cuboid::new(Vector3::new(1.0, 2.0, 1.0));
    let ray_inside = Ray::new(Point3::origin(), Vector3::y());
    let ray_miss = Ray::new(Point3::new(2.0, 2.0, 2.0), Vector3::new(1.0, 1.0, 1.0));

    // Solid cast.
    assert_eq!(
        cuboid
            .toi_with_ray(&Isometry3::identity(), &ray_inside, std::f32::MAX, true)
            .unwrap(),
        0.0
    );

    // Non-solid cast.
    assert_eq!(
        cuboid
            .toi_with_ray(&Isometry3::identity(), &ray_inside, std::f32::MAX, false)
            .unwrap(),
        2.0
    );

    // The other ray does not intersect this shape.
    assert!(cuboid
        .toi_with_ray(&Isometry3::identity(), &ray_miss, std::f32::MAX, false)
        .is_none());
    assert!(cuboid
        .toi_with_ray(&Isometry3::identity(), &ray_miss, std::f32::MAX, true)
        .is_none());
}
