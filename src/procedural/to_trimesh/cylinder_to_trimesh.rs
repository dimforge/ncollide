use std::num::Zero;
use nalgebra::na;
use geom::Cylinder;
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Vect};

impl ToTriMesh<u32> for Cylinder {
    fn to_trimesh(&self, nsubdivs: u32) -> TriMesh<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Mesh generation of cylinders with rounded corners are not implemented yet.");

        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);

        procedural::cylinder(diameter, height, nsubdivs)
    }
}
