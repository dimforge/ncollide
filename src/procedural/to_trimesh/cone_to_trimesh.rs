use na::{Pnt3, Vec3};
use na;
use geom::{Cone3, Cone3d};
use procedural::{TriMesh, ToTriMesh};
use procedural;

macro_rules! impl_to_trimesh_cone3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, u32> for $t {
            fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                // FIXME, inconsistancy we should be able to work directly with the radius.
                // FIXME, inconsistancy we should be able to work directly with the half height.
                let diameter = self.radius() * na::cast(2.0f64);
                let height   = self.half_height() * na::cast(2.0f64);

                procedural::cone(diameter, height, nsubdiv)
            }
        }
    }
)

impl_to_trimesh_cone3!(Cone3, f32)
impl_to_trimesh_cone3!(Cone3d, f64)
