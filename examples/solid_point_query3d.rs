extern crate nalgebra as na;
extern crate ncollide;

use na::{Identity, Point3, Vector3};
use ncollide::shape::Cuboid;
use ncollide::query::PointQuery;

fn main() {
    let cuboid     = Cuboid::new(Vector3::new(1.0, 2.0, 2.0));
    let pt_inside  = na::origin::<Point3<f32>>();
    let pt_outside = Point3::new(2.0, 2.0, 2.0);

    // Solid projection.
    assert_eq!(cuboid.distance_to_point(&Identity::new(), &pt_inside, true), 0.0);

    // Non-solid projection.
    assert_eq!(cuboid.distance_to_point(&Identity::new(), &pt_inside, false), -1.0);

    // The other point is outside of the cuboid so the `solid` flag has no effect.
    assert_eq!(cuboid.distance_to_point(&Identity::new(), &pt_outside, false), 1.0);
    assert_eq!(cuboid.distance_to_point(&Identity::new(), &pt_outside, true), 1.0);
}
