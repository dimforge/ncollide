use procedural::TriMesh;
use math::{Scalar, Vect};

/// Trait implemented by geometries that can be approximated by a triangle mesh.
pub trait ToTriMesh<I>
{
    /// Builds a triangle mesh from this geometry.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_trimesh(&self, i: I) -> TriMesh<Scalar, Vect>;
}
