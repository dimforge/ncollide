use super::ToTriMesh;
use crate::procedural;
use crate::procedural::TriMesh;
use crate::shape::Cone;
use alga::general::RealField;
use na;

impl<N: RealField> ToTriMesh<N> for Cone<N> {
    type DiscretizationParameter = u32;

    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<N> {
        // FIXME, inconsistancy we should be able to work directly with the radius.
        // FIXME, inconsistancy we should be able to work directly with the half height.
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);

        procedural::cone(diameter, height, nsubdiv)
    }
}
