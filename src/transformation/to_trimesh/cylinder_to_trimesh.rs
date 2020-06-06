use super::ToTriMesh;
use crate::procedural;
use crate::procedural::TriMesh;
use crate::shape::Cylinder;
use na;
use simba::scalar::RealField;

impl<N: RealField> ToTriMesh<N> for Cylinder<N> {
    type DiscretizationParameter = u32;

    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<N> {
        let diameter = self.radius * na::convert(2.0f64);
        let height = self.half_height * na::convert(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
