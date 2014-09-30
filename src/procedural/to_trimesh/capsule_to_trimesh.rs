use na;
use geom::Capsule;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::{Scalar, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<(u32, u32)> for Capsule {
    fn to_trimesh(&self, (ntheta_subdiv, nphi_subdiv): (u32, u32)) -> TriMesh<Scalar, Vect> {
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);
        // FIXME: the fact `capsule` does not take directly the half_height and the radius feels
        // inconsistant.
        procedural::capsule(&diameter, &height, ntheta_subdiv, nphi_subdiv)
    }
}

/*
#[cfg(not(3d))]
impl ToTriMesh<uint> for Capsule
{
    fn to_trimesh(&self, ntheta_subdiv: uint) -> TriMesh<Scalar, Vect>
    {
        // FIXME: generate a 2d rasterization of a capsule.
    }
}
*/
