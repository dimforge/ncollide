use std::num::Zero;
use na;
use geom::{Cone, Ball, MinkowskiSum};
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Vect};

#[dim3]
impl ToTriMesh<u32> for Cone {
    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<Scalar, Vect> {
        // FIXME, inconsistancy we should be able to work directly with the radius.
        // FIXME, inconsistancy we should be able to work directly with the half height.
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);

        procedural::cone(diameter, height, nsubdiv)
    }
}
