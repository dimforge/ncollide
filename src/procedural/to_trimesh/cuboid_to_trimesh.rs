use na;
use geom::Cuboid;
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Point, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<()> for Cuboid {
    fn to_trimesh(&self, _: ()) -> TriMesh<Scalar, Point, Vect> {
        let _2: Scalar = na::cast(2.0f64);

        procedural::cuboid(&(self.half_extents() * _2))
    }
}


// FIXME: in 2d, generate a filled rectangle.
