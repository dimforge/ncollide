use na::{Pnt3, Vec3};
use na;
use shape::{Cylinder3, Cylinder3d};
use procedural::{TriMesh, ToTriMesh};
use procedural;

macro_rules! impl_to_trimesh_cylinder3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, u32> for $t {
            fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                let diameter = self.radius() * na::cast(2.0f64);
                let height   = self.half_height() * na::cast(2.0f64);

                procedural::cylinder(diameter, height, nsubdiv)
            }
        }
    }
)

impl_to_trimesh_cylinder3!(Cylinder3, f32)
impl_to_trimesh_cylinder3!(Cylinder3d, f64)
