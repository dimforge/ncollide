use math::Point;
use trimesh::TriMesh;

/// A sample point and its associated tangent.
pub enum PathSample<P: Point> {
    /// A point that starts a new path.
    StartPoint(P, P::Vect),
    /// A point that is inside of the path currently generated.
    InnerPoint(P, P::Vect),
    /// A point that ends the path currently generated.
    EndPoint(P, P::Vect),
    /// Used when the sampler does not have any other points to generate.
    EndOfSample

}

/// A curve sampler.
pub trait CurveSampler<P: Point> {
    /// Returns the next sample point.
    fn next(&mut self) -> PathSample<P>;
}

/// A pattern that is replicated along a path.
///
/// It is responsible of the generation of the whole mesh.
pub trait StrokePattern<P: Point> {
    /// Generates the mesh using this pattern and the curve sampled by `sampler`.
    fn stroke<C: CurveSampler<P>>(&mut self, sampler: &mut C) -> TriMesh<P>;
}
