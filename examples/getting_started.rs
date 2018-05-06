extern crate nalgebra as na;
extern crate ncollide3d;

use na::{Point3, Vector3, Isometry3};
use ncollide3d::shape::Cuboid;
use ncollide3d::query::{Ray, RayCast};

fn main() {
    let cube = Cuboid::new(Vector3::new(1.0f32, 1.0, 1.0));
    let ray = Ray::new(Point3::new(0.0f32, 0.0, -1.0), Vector3::z());

    assert!(cube.intersects_ray(&Isometry3::identity(), &ray));
}
