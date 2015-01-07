//! Traits for support mapping based shapes.

/// Traits of convex shapes representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait SupportMap<P, V, M>: PreferedSamplingDirections<V, M> {
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

/// Trait of shapes having prefered sampling directions for the Minkowski sampling algorithm.
///
/// Those directions are usually the shape faces normals.
pub trait PreferedSamplingDirections<V, M> {
    /// Applies a function to this shape with a given transform.
    fn sample(&self, &M, &mut FnMut(V));
}
