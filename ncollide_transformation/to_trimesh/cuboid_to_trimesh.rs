use na::Point3;
use na;
use math::Scalar;
use geometry::shape::Cuboid3;
use procedural::TriMesh3;
use procedural;
use super::ToTriMesh;

impl<N: Scalar> ToTriMesh<Point3<N>, ()> for Cuboid3<N> {
    fn to_trimesh(&self, _: ()) -> TriMesh3<N> {
        let _2: N = na::cast(2.0f64);

        procedural::cuboid(&(*self.half_extents() * _2))
    }
}

// FIXME: in 2d, generate a filled rectangle.
