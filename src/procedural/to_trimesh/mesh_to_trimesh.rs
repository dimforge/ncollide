use na::{Pnt3, Vec3};
use shape::Mesh3;
use procedural::{ToTriMesh, TriMesh, TriMesh3, IndexBuffer};
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, ()> for Mesh3<N> {
    fn to_trimesh(&self, _: ()) -> TriMesh3<N> {
        let mut formated_index_buffer = Vec::with_capacity(self.indices().len() / 3);

        for ids in self.indices().as_slice().chunks(3) {
            formated_index_buffer.push(Vec3::new(ids[0] as u32, ids[1] as u32, ids[2] as u32))
        }

        TriMesh::new(self.vertices().deref().clone(),
        self.normals().as_ref().map(|ns| ns.deref().clone()),
        self.uvs().as_ref().map(|ns| ns.deref().clone()),
        Some(IndexBuffer::Unified(formated_index_buffer)))
    }
}
