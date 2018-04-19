use alga::general::Real;
use na::Point3;
use na;
use shape::Capsule;
use procedural::TriMesh;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for Capsule<N> {
    type DiscretizationParameter = (u32, u32);

    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<N> {
        let diameter = self.radius() * na::convert(2.0f64);
        let height = self.half_height() * na::convert(2.0f64);
        // FIXME: the fact `capsule` does not take directly the half_height and the radius feels
        // inconsistant.
        procedural::capsule(&diameter, &height, ntheta_subdiv, nphi_subdiv)
    }
}
