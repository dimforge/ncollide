use alga::general::Real;
use na::Point3;
use na;
use shape::Cylinder;
use procedural::TriMesh;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for Cylinder<N> {
    type DiscretizationParameter = u32;

    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<N> {
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
