use na::{Pnt3, Vec3};
use shape::BezierSurface3;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, (u32, u32)> for BezierSurface3<N> {
    fn to_trimesh(&self, (nusubdivs, nvsubdivs): (u32, u32)) -> TriMesh<N, Pnt3<N>, Vec3<N>> {
        procedural::bezier_surface(self.control_points(),
                                   self.nupoints(),   self.nvpoints(),
                                   nusubdivs as uint, nvsubdivs as uint)
    }
}
