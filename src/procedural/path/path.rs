use procedural::TriMesh;

/// A sample point and its associated tangent.
pub enum PathSample<P, V> {
    /// A point that starts a new path.
    StartPoint(P, V),
    /// A point that is inside of the path currently generated.
    InnerPoint(P, V),
    /// A point that ends the path currently generated.
    EndPoint(P, V),
    /// Used when the sampler does not have any other points to generate.
    EndOfSample

}

/// A curve sampler.
pub trait CurveSampler<N, P, V> {
    /// Returns the next sample point.
    fn next(&mut self) -> PathSample<P, V>;
}

/// A pattern that is replicated along a path.
///
/// It is responsible of the generation of the whole mesh.
pub trait StrokePattern<N, P, V> {
    /// Generates the mesh using this pattern and the curve sampled by `sampler`.
    fn stroke<C: CurveSampler<N, P, V>>(&mut self, sampler: &mut C) -> TriMesh<N, P, V>;
}
