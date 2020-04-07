use crate::procedural::Polyline;
use na::RealField;

/// Trait implemented by shapes that can be approximated by a triangle mesh.
pub trait ToPolyline<N: RealField> {
    type DiscretizationParameter;

    /// Builds a triangle mesh from this shape.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_polyline(&self, i: Self::DiscretizationParameter) -> Polyline<N>;
}
