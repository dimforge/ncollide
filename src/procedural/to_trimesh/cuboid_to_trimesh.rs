use std::num::Zero;
use nalgebra::na;
use geom::Cuboid;
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Vect};


impl ToTriMesh<()> for Cuboid {
    fn to_trimesh(&self, _: ()) -> TriMesh<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Rounded cuboid mesh generation is not implemented yet.");
        let _2: Scalar = na::cast(2.0f64);

        procedural::cuboid(&(self.half_extents() * _2))
    }
}


// FIXME: in 2d, generate a filled rectangle.
