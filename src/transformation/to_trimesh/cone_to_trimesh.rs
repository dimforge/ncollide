use alga::general::Real;
use na::Point3;
use na;
use shape::Cone;
use procedural::TriMesh;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for Cone<N> {
    type DiscretizationParameter = u32;

    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<N> {
        // FIXME, inconsistancy we should be able to work directly with the radius.
        // FIXME, inconsistancy we should be able to work directly with the half height.
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);

        procedural::cone(diameter, height, nsubdiv)
    }
}
