use std::sync::Arc;
use std::rand::Rng;
use na::{Vec3, Pnt2, Pnt3};
use ncollide::shape::{Mesh, Mesh3};

pub fn generate_trimesh_around_origin<R: Rng>(rng: &mut R) -> Mesh3 {
    let pts     = Vec::from_fn(3000, |_| rng.gen::<Pnt3<f32>>() * 3.0);
    let normals = Vec::from_fn(3000, |_| rng.gen::<Vec3<f32>>() * 3.0);
    let uvs     = Vec::from_fn(3000, |_| rng.gen::<Pnt2<f32>>() * 3.0);
    let indices = Vec::from_fn(3000, |i| i);

    Mesh::new(Arc::new(pts), Arc::new(indices), Some(Arc::new(uvs)), Some(Arc::new(normals)))
}
