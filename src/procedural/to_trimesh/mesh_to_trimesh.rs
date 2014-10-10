use na::Vec3;
use procedural::{ToTriMesh, TriMesh, UnifiedIndexBuffer};
use geom::Mesh;
use math::{Scalar, Point, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<()> for Mesh {
    fn to_trimesh(&self, _: ()) -> TriMesh<Scalar, Point, Vect> {
        let mut formated_index_buffer = Vec::with_capacity(self.indices().len() / 3);

        for ids in self.indices().as_slice().chunks(3) {
            formated_index_buffer.push(Vec3::new(ids[0] as u32, ids[1] as u32, ids[2] as u32))
        }

        TriMesh::new(self.vertices().deref().clone(),
                     self.normals().as_ref().map(|ns| ns.deref().clone()),
                     self.uvs().as_ref().map(|ns| ns.deref().clone()),
                     Some(UnifiedIndexBuffer(formated_index_buffer)))
                     
    }
}
