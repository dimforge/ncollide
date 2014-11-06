extern crate "nalgebra" as na;
extern crate kiss3d;
extern crate ncollide;

use std::rand;
use std::rc::Rc;
use std::cell::RefCell;
use std::sync::{Arc, RWLock};
use na::{Pnt3, Vec3, Translation};
use ncollide::procedural::path::{PolylinePath, PolylinePattern, StrokePattern, NoCap};
use ncollide::procedural;
use kiss3d::window::Window;
use kiss3d::light;
use kiss3d::resource::{ElementArrayBuffer, StaticDraw, GPUVector, Mesh};

fn main() {
    /*
     * Path stroke.
     */
    let control_points = [
        Pnt3::new(0.0f32, 1.0, 0.0),
        Pnt3::new(2.0f32, 4.0, 2.0),
        Pnt3::new(2.0f32, 1.0, 4.0),
        Pnt3::new(4.0f32, 4.0, 6.0),
        Pnt3::new(2.0f32, 1.0, 8.0),
        Pnt3::new(2.0f32, 4.0, 10.0),
        Pnt3::new(0.0f32, 1.0, 12.0),
        Pnt3::new(-2.0f32, 4.0, 10.0),
        Pnt3::new(-2.0f32, 1.0, 8.0),
        Pnt3::new(-4.0f32, 4.0, 6.0),
        Pnt3::new(-2.0f32, 1.0, 4.0),
        Pnt3::new(-2.0f32, 4.0, 2.0),
    ];
    let bezier      = procedural::bezier_curve(control_points, 100);
    let mut path    = PolylinePath::new(&bezier);
    let pattern     = procedural::unit_circle(100);
    let mut pattern = PolylinePattern::new(&pattern, true, NoCap::new(), NoCap::new());
    let mut trimesh = pattern.stroke(&mut path);

    trimesh.recompute_normals();


    /*
     * Decomposition of the mesh.
     */
    let (decomp, partitioning) = procedural::hacd(trimesh.clone(), 0.03, 0);

    /*
     * Rendering.
     */
    let mut window = Window::new("hacd demo");

    window.set_background_color(1.0, 1.0, 1.0);

    let mut m = window.add_trimesh(trimesh, na::one());
    m.enable_backface_culling(false);

    let coords  = m.data().object().expect("here").mesh().borrow().coords().clone();
    let normals = m.data().object().unwrap().mesh().borrow().normals().clone();
    let uvs     = m.data().object().unwrap().mesh().borrow().uvs().clone();
    let faces   = m.data().object().unwrap().mesh().borrow().faces().clone();


    for (comp, partitioning) in decomp.into_iter().zip(partitioning.into_iter()) {
        let r = rand::random();
        let g = rand::random();
        let b = rand::random();

        let mut m  = window.add_trimesh(comp.clone(), na::one());
        m.set_color(r, g, b);
        m.append_translation(&Vec3::new(-15.0, 0.0, 0.0));

        let mut part_faces = Vec::new();

        for i in partitioning.into_iter() {
            part_faces.push(faces.read().data().as_ref().unwrap()[i]);
        }

        let faces = Arc::new(RWLock::new(GPUVector::new(part_faces, ElementArrayBuffer, StaticDraw)));

        let mesh = Mesh::new_with_gpu_vectors(coords.clone(), faces, normals.clone(), uvs.clone());
        let mesh = Rc::new(RefCell::new(mesh));
        let mut m  = window.add_mesh(mesh, na::one());

        m.set_color(r, g, b);
        m.append_translation(&Vec3::new(-7.5, 0.0, 0.0));
        m.enable_backface_culling(false);
    }

    window.set_light(light::StickToCamera);

    while window.render() {
    }
}
