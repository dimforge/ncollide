extern crate nalgebra as na;
extern crate ncollide;

use na::{Identity, Pnt3, Vec3};
use ncollide::shape::Cuboid;
use ncollide::ray::{Ray, RayCast};

fn main() {
    let cuboid     = Cuboid::new(Vec3::new(1.0, 2.0, 1.0));
    let ray_inside = Ray::new(na::orig::<Pnt3<f32>>(), Vec3::y());
    let ray_miss   = Ray::new(Pnt3::new(2.0, 2.0, 2.0), Vec3::new(1.0, 1.0, 1.0));

    assert!(cuboid.toi_with_ray(&Identity::new(), &ray_inside, true).unwrap()  == 0.0); // solid cast.
    assert!(cuboid.toi_with_ray(&Identity::new(), &ray_inside, false).unwrap() == 2.0); // non-solid cast.

    // The other ray does not intersect this shape.
    assert!(cuboid.toi_with_ray(&Identity::new(), &ray_miss, false).is_none());
    assert!(cuboid.toi_with_ray(&Identity::new(), &ray_miss, true).is_none());
}
