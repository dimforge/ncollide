extern crate "nalgebra" as na;
extern crate ncollide;

use na::{Pnt2, Vec2};
use ncollide::shape::Cuboid;
use ncollide::ray::{Ray, LocalRayCast};

fn main() {
    let cuboid     = Cuboid::new(Vec2::new(1.0f32, 2.0));
    let ray_inside = Ray::new(na::orig::<Pnt2<f32>>(), Vec2::y());
    let ray_miss   = Ray::new(Pnt2::new(2.0, 2.0), Vec2::new(1.0, 1.0));

    assert!(cuboid.toi_with_ray(&ray_inside, true).unwrap()  == 0.0); // solid cast.
    assert!(cuboid.toi_with_ray(&ray_inside, false).unwrap() == 2.0); // non-solid cast.

    // The other ray does not intersect this shape.
    assert!(cuboid.toi_with_ray(&ray_miss, false).is_none());
    assert!(cuboid.toi_with_ray(&ray_miss, true).is_none());
}
