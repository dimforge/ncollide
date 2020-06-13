use super::ToTriMesh;
use crate::procedural;
use crate::procedural::TriMesh;
use crate::shape::Ball;
use na;
use simba::scalar::RealField;

impl<N: RealField> ToTriMesh<N> for Ball<N> {
    type DiscretizationParameter = (u32, u32);

    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<N> {
        procedural::sphere(
            self.radius() * na::convert(2.0f64),
            ntheta_subdiv,
            nphi_subdiv,
            true,
        )
    }
}
