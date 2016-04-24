extern crate nalgebra as na;
extern crate ncollide;

use ncollide::inspection::Shape;
use ncollide::shape::{Ball, ShapeHandle2};

fn main() {
    let shape = ShapeHandle2::new(Ball::new(1.0f32));

    assert!(shape.desc().is_shape::<Ball<f32>>());
    assert!(shape.desc().is_support_map());
}
