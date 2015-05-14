use na;
use na::Pnt3;
use math::Scalar;
use entities::shape;
use procedural::{TriMesh, TriMesh3, IndexBuffer};
use super::ToTriMesh;

impl<N: Scalar> ToTriMesh<Pnt3<N>, ()> for shape::TriMesh3<N> {
    fn to_trimesh(&self, _: ()) -> TriMesh3<N> {
        TriMesh::new((**self.vertices()).clone(),
        self.normals().as_ref().map(|ns| (**ns).clone()),
        self.uvs().as_ref().map(|ns| (**ns).clone()),
        Some(IndexBuffer::Unified((**self.indices()).iter().map(|e| na::cast(*e)).collect())))
    }
}
