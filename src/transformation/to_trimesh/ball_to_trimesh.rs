use alga::general::Real;
use na;
use shape::Ball;
use procedural::TriMesh;
use procedural;
use super::ToTriMesh;

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
