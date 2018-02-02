extern crate nalgebra as na;
extern crate ncollide;

use na::{Id, Point2, Vector2};
use ncollide::shape::Cuboid;
use ncollide::query::PointQuery;

fn main() {
    let cuboid = Cuboid::new(Vector2::new(1.0, 2.0));
    let pt_inside = na::origin::<Point2<f32>>();
    let pt_outside = Point2::new(2.0, 2.0);

    // Solid projection.
    assert_eq!(cuboid.distance_to_point(&Id::new(), &pt_inside, true), 0.0);

    // Non-solid projection.
    assert_eq!(
        cuboid.distance_to_point(&Id::new(), &pt_inside, false),
        -1.0
    );

    // The other point is outside of the cuboid so the `solid` flag has no effect.
    assert_eq!(
        cuboid.distance_to_point(&Id::new(), &pt_outside, false),
        1.0
    );
    assert_eq!(cuboid.distance_to_point(&Id::new(), &pt_outside, true), 1.0);
}
