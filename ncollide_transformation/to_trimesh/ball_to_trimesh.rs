use alga::general::Real;
use na::Point3;
use na;
use geometry::shape::Ball3;
use procedural::TriMesh3;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<Point3<N>, (u32, u32)> for Ball3<N> {
    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh3<N> {
        procedural::sphere(
            self.radius() * na::convert(2.0f64),
            ntheta_subdiv,
            nphi_subdiv,
            true,
        )
    }
}
