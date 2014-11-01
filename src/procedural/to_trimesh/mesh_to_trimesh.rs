use na::{Pnt3, Vec3};
use shape::{Mesh3, Mesh3d};
use procedural::{ToTriMesh, TriMesh, UnifiedIndexBuffer};

macro_rules! impl_to_trimesh_mesh3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, ()> for $t {
            fn to_trimesh(&self, _: ()) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
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
    }
)

impl_to_trimesh_mesh3!(Mesh3, f32)
impl_to_trimesh_mesh3!(Mesh3d, f64)
