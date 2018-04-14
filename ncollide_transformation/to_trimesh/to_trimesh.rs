use procedural::TriMesh;
use math::Point;

/// Trait implemented by shapes that can be approximated by a triangle mesh.
pub trait ToTriMesh<N: Real, I> {
    /// Builds a triangle mesh from this shape.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_trimesh(&self, i: I) -> TriMesh<P>;
}
