use procedural::Polyline;
use math::Point;

/// Trait implemented by shapes that can be approximated by a triangle mesh.
pub trait ToPolyline<N: Real, I> {
    /// Builds a triangle mesh from this shape.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_polyline(&self, i: I) -> Polyline<P>;
}
