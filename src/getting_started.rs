extern crate "nalgebra" as na;
extern crate "ncollide3df32" as ncollide;

use na::{Pnt3, Vec3};
use ncollide::geom::Cuboid;
use ncollide::ray::{Ray, RayCast};

fn main() {
    let cube = Cuboid::new(Vec3::new(1.0f32, 1.0, 1.0));
    let ray  = Ray::new(Pnt3::new(0.0f32, 0.0, -1.0), Vec3::z());

    assert!(cube.intersects_ray(&ray));
}
