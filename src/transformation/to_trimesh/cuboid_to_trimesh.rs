use alga::general::Real;
use na;
use shape::Cuboid;
use procedural::TriMesh;
use procedural;
use super::ToTriMesh;

impl<N: Real> ToTriMesh<N> for Cuboid<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let _2: N = na::convert(2.0f64);

        procedural::cuboid(&(*self.half_extents() * _2))
    }
}

// FIXME: in 2d, generate a filled rectangle.
