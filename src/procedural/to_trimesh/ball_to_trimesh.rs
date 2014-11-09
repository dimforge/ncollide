use na::{Pnt3, Vec3};
use na;
use shape::Ball3;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, (u32, u32)> for Ball3<N> {
    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<N, Pnt3<N>, Vec3<N>> {
        procedural::sphere(self.radius() * na::cast(2.0f64), ntheta_subdiv, nphi_subdiv, true)
    }
}
