use na::{Pnt3, Vec3};
use na;
use entities::shape::Capsule3;
use procedural::TriMesh3;
use procedural;
use super::ToTriMesh;
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, (u32, u32)> for Capsule3<N> {
    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh3<N> {
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);
        // FIXME: the fact `capsule` does not take directly the half_height and the radius feels
        // inconsistant.
        procedural::capsule(&diameter, &height, ntheta_subdiv, nphi_subdiv)
    }
}
