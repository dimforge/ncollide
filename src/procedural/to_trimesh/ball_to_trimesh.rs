use nalgebra::na;
use geom::Ball;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::{Scalar, Vect};

impl ToTriMesh<(u32, u32)> for Ball {
    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<Scalar, Vect> {
        procedural::sphere(&(self.radius() * na::cast(2.0f64)), ntheta_subdiv, nphi_subdiv, true)
    }
}

/*
impl ToTriMesh<uint> for Ball
{
    fn to_trimesh(&self, ntheta_subdiv: uint) -> TriMesh<Scalar, Vect>
    {
        // XXX: have something that generate a circle as a triangle mesh.
        // (NOTE: having this would simplify the cylinder generator too!
    }
}
*/
