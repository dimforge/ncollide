extern crate rand;
extern crate nalgebra as na;
extern crate kiss3d;
extern crate ncollide;

use std::rc::Rc;
use std::cell::RefCell;
use std::sync::{Arc, RwLock};
use na::{Point3, Vector3, Translation};
use ncollide::procedural::path::{PolylinePath, PolylinePattern, StrokePattern, NoCap};
use ncollide::transformation;
use ncollide::procedural;
use kiss3d::window::Window;
use kiss3d::light::Light;
use kiss3d::resource::{BufferType, AllocationType, GPUVec, Mesh};

fn main() {
    /*
     * Path stroke.
     */
    let control_points = [
        Point3::new(0.0f32, 1.0, 0.0),
        Point3::new(2.0f32, 4.0, 2.0),
        Point3::new(2.0f32, 1.0, 4.0),
        Point3::new(4.0f32, 4.0, 6.0),
        Point3::new(2.0f32, 1.0, 8.0),
        Point3::new(2.0f32, 4.0, 10.0),
        Point3::new(0.0f32, 1.0, 12.0),
        Point3::new(-2.0f32, 4.0, 10.0),
        Point3::new(-2.0f32, 1.0, 8.0),
        Point3::new(-4.0f32, 4.0, 6.0),
        Point3::new(-2.0f32, 1.0, 4.0),
        Point3::new(-2.0f32, 4.0, 2.0),
    ];
    let bezier      = procedural::bezier_curve(&control_points, 100);
    let mut path    = PolylinePath::new(&bezier);
    let pattern     = procedural::unit_circle(100);
    let mut pattern = PolylinePattern::new(&pattern, true, NoCap::new(), NoCap::new());
    let mut trimesh = pattern.stroke(&mut path);

    trimesh.recompute_normals();


    /*
     * Decomposition of the mesh.
     */
    let (decomp, partitioning) = transformation::hacd(trimesh.clone(), 0.03, 0);

    /*
     * Rendering.
     */
    let mut window = Window::new("hacd demo");

    window.set_background_color(1.0, 1.0, 1.0);

    let mut m = window.add_trimesh(trimesh, na::one());
    m.enable_backface_culling(false);

    {
        let mdata   = m.data();
        let mesh    = mdata.object().unwrap().mesh();
        let coords  = mesh.borrow().coords().clone();
        let normals = mesh.borrow().normals().clone();
        let uvs     = mesh.borrow().uvs().clone();
        let faces   = mesh.borrow().faces().clone();


        for (comp, partitioning) in decomp.into_iter().zip(partitioning.into_iter()) {
            let r = rand::random();
            let g = rand::random();
            let b = rand::random();

            let mut m  = window.add_trimesh(comp.clone(), na::one());
            m.set_color(r, g, b);
            m.append_translation(&Vector3::new(-15.0, 0.0, 0.0));

            let mut part_faces = Vec::new();

            for i in partitioning.into_iter() {
                part_faces.push(faces.read().unwrap().data().as_ref().unwrap()[i]);
            }

            let faces = Arc::new(RwLock::new(GPUVec::new(part_faces, BufferType::ElementArray, AllocationType::StaticDraw)));

            let mesh = Mesh::new_with_gpu_vectors(coords.clone(), faces, normals.clone(), uvs.clone());
            let mesh = Rc::new(RefCell::new(mesh));
            let mut m  = window.add_mesh(mesh, na::one());

            m.set_color(r, g, b);
            m.append_translation(&Vector3::new(-7.5, 0.0, 0.0));
            m.enable_backface_culling(false);
        }
    }

    window.set_light(Light::StickToCamera);

    while window.render() {
    }
}
