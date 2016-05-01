extern crate nalgebra as na;
extern crate ncollide;

use na::{Identity, Point3, Vector3};
use ncollide::shape::Cuboid;
use ncollide::query::{Ray, RayCast};

fn main() {
    let cube = Cuboid::new(Vector3::new(1.0f32, 1.0, 1.0));
    let ray  = Ray::new(Point3::new(0.0f32, 0.0, -1.0), Vector3::z());

    assert!(cube.intersects_ray(&Identity::new(), &ray));
}
