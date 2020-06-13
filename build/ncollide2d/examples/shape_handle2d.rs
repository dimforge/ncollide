extern crate nalgebra as na;

use ncollide2d::shape::{Ball, ShapeHandle};

fn main() {
    let shape = ShapeHandle::new(Ball::new(1.0f32));

    assert!(shape.is_shape::<Ball<f32>>());
    assert!(shape.is_support_map());
}
