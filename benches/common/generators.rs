use std::sync::Arc;
use std::rand::Rng;
use na::{Vec3, Pnt2, Pnt3};
use ncollide::shape::{TriMesh, TriMesh3};

pub fn generate_trimesh_around_origin<R: Rng>(rng: &mut R) -> TriMesh3<f32> {
    let pts     = (0 .. 3000).map(|_| rng.gen::<Pnt3<f32>>() * 3.0).collect();
    let normals = (0 .. 3000).map(|_| rng.gen::<Vec3<f32>>() * 3.0).collect();
    let uvs     = (0 .. 3000).map(|_| rng.gen::<Pnt2<f32>>() * 3.0).collect();
    let indices = (0 .. 1000).map(|i| Pnt3::new(i * 3, i * 3 + 1, i * 3 + 2)).collect();

    TriMesh::new(Arc::new(pts), Arc::new(indices), Some(Arc::new(uvs)), Some(Arc::new(normals)))
}
