use na::Real;

use procedural::TriMesh;

/// Trait implemented by shapes that can be approximated by a triangle mesh.
pub trait ToTriMesh<N: Real> {
    type DiscretizationParameter;

    /// Builds a triangle mesh from this shape.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_trimesh(&self, i: Self::DiscretizationParameter) -> TriMesh<N>;
}
