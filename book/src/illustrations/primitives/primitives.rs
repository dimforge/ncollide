extern crate native;
extern crate "ncollide3df32" as ncollide;
extern crate kiss3d;
extern crate nalgebra;

use std::rand;
use nalgebra::na;
use nalgebra::na::Vec3;
use ncollide::procedural::TriMesh;
use ncollide::procedural;
use kiss3d::window::Window;
use kiss3d::light;

#[start]
fn start(argc: int, argv: *const *const u8) -> int {
    native::start(argc, argv, main)
}

fn main() {
    let mut window = Window::new("Kiss3d: procedural");

    let mut points = Vec::new();
    for _ in range(0u, 100000) {
        points.push(rand::random::<Vec3<f32>>() * 2.0f32);
    }

    let convex_hull = procedural::convex_hull3d(points.as_slice());

    /*
     * Rendering.
     */
    let mut mhull = window.add_trimesh(convex_hull, na::one());
    let mut mpts  = window.add_trimesh(TriMesh::new(points, None, None, None), na::one());
    mhull.set_color(0.0, 1.0, 0.0);
    mhull.set_lines_width(6.0);
    mhull.set_surface_rendering_activation(false);
    mhull.set_points_size(10.0);
    mpts.set_color(0.0, 0.0, 1.0);
    mpts.set_points_size(2.0);
    mpts.set_surface_rendering_activation(false);

    window.set_background_color(1.0, 1.0, 1.0);
    window.set_light(light::StickToCamera);

    while window.render() {
    }
}
