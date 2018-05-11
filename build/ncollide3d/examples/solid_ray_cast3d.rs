extern crate nalgebra as na;
extern crate ncollide3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::shape::Cuboid;
use ncollide3d::query::{Ray, RayCast};

fn main() {
    let cuboid = Cuboid::new(Vector3::new(1.0, 2.0, 1.0));
    let ray_inside = Ray::new(na::origin::<Point3<f32>>(), Vector3::y());
    let ray_miss = Ray::new(Point3::new(2.0, 2.0, 2.0), Vector3::new(1.0, 1.0, 1.0));

    // Solid cast.
    assert!(cuboid.toi_with_ray(&Isometry3::identity(), &ray_inside, true).unwrap() == 0.0);

    // Non-solid cast.
    assert!(cuboid.toi_with_ray(&Isometry3::identity(), &ray_inside, false).unwrap() == 2.0);

    // The other ray does not intersect this shape.
    assert!(cuboid.toi_with_ray(&Isometry3::identity(), &ray_miss, false).is_none());
    assert!(cuboid.toi_with_ray(&Isometry3::identity(), &ray_miss, true).is_none());
}
