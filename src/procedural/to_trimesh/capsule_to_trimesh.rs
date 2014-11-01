use na::{Pnt3, Vec3};
use na;
use shape::{Capsule3, Capsule3d};
use procedural::{ToTriMesh, TriMesh};
use procedural;

macro_rules! impl_to_trimesh_capsule3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, (u32, u32)> for $t {
            fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                let diameter = self.radius() * na::cast(2.0f64);
                let height   = self.half_height() * na::cast(2.0f64);
                // FIXME: the fact `capsule` does not take directly the half_height and the radius feels
                // inconsistant.
                procedural::capsule(&diameter, &height, ntheta_subdiv, nphi_subdiv)
            }
        }
    }
)

impl_to_trimesh_capsule3!(Capsule3, f32)
impl_to_trimesh_capsule3!(Capsule3d, f64)
