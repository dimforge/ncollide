use na::{Pnt3, Vec3};
use shape::{BezierSurface3, BezierSurface3d};
use procedural::{ToTriMesh, TriMesh};
use procedural;

macro_rules! impl_to_trimesh_bezier_surface3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, (u32, u32)> for $t {
            fn to_trimesh(&self, (nusubdivs, nvsubdivs): (u32, u32)) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                procedural::bezier_surface(self.control_points(),
                                           self.nupoints(),   self.nvpoints(),
                                           nusubdivs as uint, nvsubdivs as uint)
            }
        }
    }
)

impl_to_trimesh_bezier_surface3!(BezierSurface3, f32)
impl_to_trimesh_bezier_surface3!(BezierSurface3d, f64)
