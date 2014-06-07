use procedural::Polyline;
use math::{Scalar, Vect};

/// Trait implemented by geometries that can be approximated by a triangle mesh.
pub trait ToPolyline<I>
{
    /// Builds a triangle mesh from this geometry.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_polyline(&self, i: I) -> Polyline<Scalar, Vect>;
}
