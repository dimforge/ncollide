use na::{Pnt3, Vec3};
use na;
use shape::Cylinder3;
use procedural::{ToTriMesh, TriMesh3};
use procedural;
use math::Scalar;

impl<N: Scalar> ToTriMesh<N, Pnt3<N>, Vec3<N>, u32> for Cylinder3<N> {
    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh3<N> {
        let diameter = self.radius() * na::cast(2.0f64);
        let height   = self.half_height() * na::cast(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
