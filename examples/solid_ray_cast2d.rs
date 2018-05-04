extern crate nalgebra as na;
extern crate ncollide2d;

use na::{Id, Point2, Vector2};
use ncollide2d::shape::Cuboid;
use ncollide2d::query::{Ray, RayCast};

fn main() {
    let cuboid = Cuboid::new(Vector2::new(1.0, 2.0));
    let ray_inside = Ray::new(na::origin::<Point2<f32>>(), Vector2::y());
    let ray_miss = Ray::new(Point2::new(2.0, 2.0), Vector2::new(1.0, 1.0));

    // Solid cast.
    assert_eq!(
        cuboid.toi_with_ray(&Isometry::identity(), &ray_inside, true).unwrap(),
        0.0
    );

    // Non-solid cast.
    assert_eq!(
        cuboid.toi_with_ray(&Isometry::identity(), &ray_inside, false).unwrap(),
        2.0
    );

    // The other ray does not intersect this shape.
    assert!(cuboid.toi_with_ray(&Isometry::identity(), &ray_miss, false).is_none());
    assert!(cuboid.toi_with_ray(&Isometry::identity(), &ray_miss, true).is_none());
}
