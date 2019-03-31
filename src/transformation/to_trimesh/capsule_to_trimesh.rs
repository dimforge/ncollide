use super::ToTriMesh;
use alga::general::RealField;
use na;
use crate::procedural;
use crate::procedural::TriMesh;
use crate::shape::Capsule;

impl<N: RealField> ToTriMesh<N> for Capsule<N> {
    type DiscretizationParameter = (u32, u32);

    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<N> {
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);
        // FIXME: the fact `capsule` does not take directly the half_height and the radius feels
        // inconsistant.
        procedural::capsule(&diameter, &height, ntheta_subdiv, nphi_subdiv)
    }
}
