use na::{Pnt3, Vec3};
use na;
use geom::{Ball3, Ball3d};
use procedural::{ToTriMesh, TriMesh};
use procedural;

macro_rules! impl_to_trimesh_ball3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, (u32, u32)> for $t {
            fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                procedural::sphere(self.radius() * na::cast(2.0f64), ntheta_subdiv, nphi_subdiv, true)
            }
        }
    }
)

impl_to_trimesh_ball3!(Ball3, f32)
impl_to_trimesh_ball3!(Ball3d, f64)
