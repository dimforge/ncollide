use std::num::Zero;
use nalgebra::na::Vec3;
use procedural::{ToTriMesh, TriMesh, UnifiedIndexBuffer};
use geom::Mesh;
use math::{Scalar, Vect};

#[dim3]
impl ToTriMesh<()> for Mesh {
    fn to_trimesh(&self, _: ()) -> TriMesh<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Mesh generation with non zero margin is not implemented yet.");

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
