use na;
use geom::Cylinder;
use procedural::{TriMesh, ToTriMesh};
use procedural;
use math::{Scalar, Point, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<u32> for Cylinder {
    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh<Scalar, Point, Vect> {
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
