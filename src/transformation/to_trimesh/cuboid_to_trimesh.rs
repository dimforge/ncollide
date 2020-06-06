use super::ToTriMesh;
use crate::procedural;
use crate::procedural::TriMesh;
use crate::shape::Cuboid;
use na;
use simba::scalar::RealField;

impl<N: RealField> ToTriMesh<N> for Cuboid<N> {
    type DiscretizationParameter = ();

    fn to_trimesh(&self, _: ()) -> TriMesh<N> {
        let _2: N = na::convert(2.0f64);

        procedural::cuboid(&(self.half_extents * _2))
    }
}

// FIXME: in 2d, generate a filled rectangle.
