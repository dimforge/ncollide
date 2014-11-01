use na::{Pnt3, Vec3};
use shape::{Triangle3, Triangle3d};
use procedural::{ToTriMesh, TriMesh};

macro_rules! impl_to_trimesh_triangle3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, ()> for $t {
            fn to_trimesh(&self, _: ()) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                TriMesh::new(
                    vec!(self.a().clone(), self.b().clone(), self.c().clone()),
                    None,
                    None,
                    None)
            }
        }
    }
)

impl_to_trimesh_triangle3!(Triangle3, f32)
impl_to_trimesh_triangle3!(Triangle3d, f64)
