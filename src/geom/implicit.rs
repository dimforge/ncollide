use nalgebra::vec::AlgebraicVec;

// Sadly, we cannot put this on the `Implicit` trait because the caller of `margin` might get
// unconstrained type.
/// Trait of geometries having a margin.
pub trait HasMargin<N> {
    /**
     * The geometry margin.
     */
    fn margin(&self) -> N;
}

/// Traits of convex geometries representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returnd point.
pub trait Implicit<N: Algebraic, V: AlgebraicVec<N>, M>: HasMargin<N>{
    /**
     * Evaluates the support function of the object. A support function is a
     * function associating a vector to the geometry point which maximizes their
     * dot product.
     * 
     * # Arguments:
     *  * `dir` - the input of the support function. It is not required for it to
     *            be normalized.
     */
    #[inline]
    fn support_point(&self, transform: &M, dir: &V) -> V {
        let wo_margin = self.support_point_without_margin(transform, dir);

        wo_margin + dir.normalized() * self.margin()
    }

    /**
     * Evaluates the support function of the object. A support function is a
     * function associating a vector to the geometry point which maximizes their
     * dot product. This does not include the `margin` of the object. Margins are
     * geometry-dependent. Use `support_point` to sample the complete geometry.
     * 
     * # Arguments:
     *  * `dir` - the input of the support function. It is not required for it to
     *            be normalized.
     */
    fn support_point_without_margin(&self, transform: &M, dir: &V) -> V;
}
