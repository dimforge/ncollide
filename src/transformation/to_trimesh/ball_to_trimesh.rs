use super::ToTriMesh;
use alga::general::Real;
use na;
use procedural;
use procedural::TriMesh;
use shape::Ball;

impl<N: Real> ToTriMesh<N> for Ball<N> {
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
