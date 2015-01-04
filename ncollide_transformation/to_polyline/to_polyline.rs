use procedural::Polyline;


/// Trait implemented by shapes that can be approximated by a triangle mesh.
pub trait ToPolyline<N, P, V, I>
{
    /// Builds a triangle mesh from this shape.
    ///
    /// # Arguments:
    /// * `i` - the discretization parameters.
    fn to_polyline(&self, i: I) -> Polyline<N, P, V>;
}
