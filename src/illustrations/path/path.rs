extern crate native;
extern crate "ncollide3df32" as ncollide;
extern crate kiss3d;
extern crate nalgebra;

use nalgebra::na::Vec3;
use ncollide::procedural::path::{PolylinePath, PolylinePattern, StrokePattern, ArrowheadCap};
use ncollide::procedural;
use kiss3d::window::Window;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    let mut window = Window::new("Kiss3d: procedural");

    let control_points = [
        Vec3::new(0.0f32, 1.0, 0.0),
        Vec3::new(2.0f32, 4.0, 2.0),
        Vec3::new(2.0f32, 1.0, 4.0),
        Vec3::new(4.0f32, 4.0, 6.0),
        Vec3::new(2.0f32, 1.0, 8.0),
        Vec3::new(2.0f32, 4.0, 10.0),
        Vec3::new(0.0f32, 1.0, 12.0),
        Vec3::new(-2.0f32, 4.0, 10.0),
        Vec3::new(-2.0f32, 1.0, 8.0),
        Vec3::new(-4.0f32, 4.0, 6.0),
        Vec3::new(-2.0f32, 1.0, 4.0),
        Vec3::new(-2.0f32, 4.0, 2.0),
    ];

    // Setup the path.
    let bezier   = procedural::bezier_curve(control_points, 100);
    let mut path = PolylinePath::new(&bezier);


    // Setup the pattern.
    let start_cap   = ArrowheadCap::new(1.5f32, 2.0, 0.0);
    let end_cap     = ArrowheadCap::new(2.0f32, 2.0, 0.5);
    let pattern     = procedural::unit_circle(100);
    let mut pattern = PolylinePattern::new(&pattern, true, start_cap, end_cap);


    // Stroke!
    let mesh = pattern.stroke(&mut path);

    /*
     * Rendering.
     */
    window.set_background_color(1.0, 1.0, 1.0);
    window.add_trimesh(mesh, Vec3::new(0.5f32, 0.5, 0.5)).set_color(1.0, 1.0, 0.0);
    window.set_light(light::StickToCamera);

    while window.render() {
    }
}
