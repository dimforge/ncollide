use std::num::Zero;
use nalgebra::na;
use geom::{Cylinder, Ball, MinkowskiSum};
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Vect};

#[dim3]
impl ToTriMesh<u32> for Cylinder {
    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<Scalar, Vect> {
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
