use procedural::TriMesh;

/// A sample point and its associated tangent.
pub enum PathSample<V> {
    /// A point that starts a new path.
    StartPoint(V, V),
    /// A point that is inside of the path currently generated.
    InnerPoint(V, V),
    /// A point that ends the path currently generated.
    EndPoint(V, V),
    /// Used when the sampler does not have any other points to generate.
    EndOfSample

}

/// A curve sampler.
pub trait CurveSampler<N, V> {
    /// Returns the next sample point.
    fn next(&mut self) -> PathSample<V>;
}

/// A pattern that is replicated along a path.
///
/// It is responsible of the generation of the whole mesh.
pub trait StrokePattern<N, V> {
    /// Generates the mesh using this pattern and the curve sampled by `sampler`.
    fn stroke<C: CurveSampler<N, V>>(&mut self, sampler: &mut C) -> TriMesh<N, V>;
}
