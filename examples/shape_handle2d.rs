extern crate nalgebra as na;
extern crate ncollide;

use ncollide::shape::{Ball, ShapeHandle2};

fn main() {
    let shape = ShapeHandle2::new(Ball::new(1.0f32));

    assert!(shape.is_shape::<Ball<f32>>());
    assert!(shape.is_support_map());
}
