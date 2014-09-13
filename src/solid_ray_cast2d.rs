extern crate nalgebra;
extern crate "ncollide2df32" as ncollide;

use nalgebra::na;
use nalgebra::na::Vec2;
use ncollide::geom::Cuboid;
use ncollide::ray::{Ray, RayCast};

fn main() {
    let cuboid     = Cuboid::new(Vec2::new(1.0, 2.0));
    let ray_inside = Ray::new(na::zero(), Vec2::y());
    let ray_miss   = Ray::new(Vec2::new(2.0, 2.0), Vec2::new(1.0, 1.0));

    assert!(cuboid.toi_with_ray(&ray_inside, true).unwrap()  == 0.0); // solid cast.
    assert!(cuboid.toi_with_ray(&ray_inside, false).unwrap() == 2.0); // non-solid cast.

    // The other ray does not intersect this shape.
    assert!(cuboid.toi_with_ray(&ray_miss, false).is_none());
    assert!(cuboid.toi_with_ray(&ray_miss, true).is_none());
}
