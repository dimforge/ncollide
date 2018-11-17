use super::ToTriMesh;
use alga::general::Real;
use na;
use procedural;
use procedural::TriMesh;
use shape::Cylinder;

impl<N: Real> ToTriMesh<N> for Cylinder<N> {
    type DiscretizationParameter = u32;

    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<N> {
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
