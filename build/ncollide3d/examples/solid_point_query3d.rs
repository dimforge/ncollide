extern crate nalgebra as na;
extern crate ncollide3d;

use na::{Isometry3, Point3, Vector3};
use ncollide3d::query::PointQuery;
use ncollide3d::shape::Cuboid;

fn main() {
    let cuboid = Cuboid::new(Vector3::new(1.0, 2.0, 2.0));
    let pt_inside = Point3::origin();
    let pt_outside = Point3::new(2.0, 2.0, 2.0);

    // Solid projection.
    assert_eq!(
        cuboid.distance_to_point(&Isometry3::identity(), &pt_inside, true),
        0.0
    );

    // Non-solid projection.
    assert_eq!(
        cuboid.distance_to_point(&Isometry3::identity(), &pt_inside, false),
        -1.0
    );

    // The other point is outside of the cuboid so the `solid` flag has no effect.
    assert_eq!(
        cuboid.distance_to_point(&Isometry3::identity(), &pt_outside, false),
        1.0
    );
    assert_eq!(
        cuboid.distance_to_point(&Isometry3::identity(), &pt_outside, true),
        1.0
    );
}
