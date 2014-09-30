use geom::BezierSurface;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::{Scalar, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<(u32, u32)> for BezierSurface {
    fn to_trimesh(&self, (nusubdivs, nvsubdivs): (u32, u32)) -> TriMesh<Scalar, Vect> {
        procedural::bezier_surface(self.control_points(),
                                   self.nupoints(),   self.nvpoints(),
                                   nusubdivs as uint, nvsubdivs as uint)
        //                            FIXME: ^^^^^^^
        //                            Typing inconsistancies.
    }
}
