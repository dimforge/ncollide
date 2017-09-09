use alga::general::Real;
use na::Point3;
use na;
use geometry::shape::Cylinder3;
use procedural::TriMesh3;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<Point3<N>, u32> for Cylinder3<N> {
    fn to_trimesh(&self, nsubdiv: u32) -> TriMesh3<N> {
        let diameter = self.radius() * na::convert(2.0f64);
        let height   = self.half_height() * na::convert(2.0f64);

        procedural::cylinder(diameter, height, nsubdiv)
    }
}
