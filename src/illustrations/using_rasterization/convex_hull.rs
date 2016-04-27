extern crate rand;
extern crate ncollide;
extern crate kiss3d;
extern crate nalgebra as na;

use na::Point3;
use ncollide::procedural::TriMesh;
use ncollide::transformation;
use kiss3d::window::Window;
use kiss3d::light::Light;

fn main() {
    let mut window = Window::new("Kiss3d: procedural");

    let mut points = Vec::new();
    for _ in 0usize .. 100000 {
        points.push(rand::random::<Point3<f32>>() * 2.0f32);
    }

    let convex_hull = transformation::convex_hull3(points.as_slice());

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
    window.set_light(Light::StickToCamera);

    while window.render() {
    }
}
