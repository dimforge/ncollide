//! Traits for support mapping based geometries.

/// Traits of convex geometries representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait Implicit<P, V, M> {
    // FIXME: add methods that takes a unit `dir` in argument.
    // This might be useful to avoid useless normalizations.
    /**
     * Evaluates the support function of the object. A support function is a
     * function associating a vector to the shape point which maximizes their
     * dot product. This does not include the `margin` of the object. Margins are
     * shape-dependent. Use `support_point` to sample the complete shape.
     *
     * # Arguments:
     *  * `dir` - the input of the support function. It is not required for it to
     *            be normalized.
     */
    fn support_point(&self, transform: &M, dir: &V) -> P;
}

impl<'a, P, V, M> Implicit<P, V, M> for &'a Implicit<P, V, M> + 'a {
    #[inline]
    fn support_point(&self, transform: &M, dir: &V) -> P {
        self.support_point(transform, dir)
    }
}

/// Trait of geometries having prefered sampling directions for the Minkowski sampling algorithm.
///
/// Those directions are usually the shape faces normals.
pub trait PreferedSamplingDirections<V, M> {
    /// Applies a function to this shape with a given transform.
    fn sample(&self, &M, |V| -> ());
}
