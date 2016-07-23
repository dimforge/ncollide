//! Traits for support mapping based shapes.

use math::{Point, Vector};

/// Traits of convex shapes representable by a support mapping function.
///
/// # Parameters:
///   * V - type of the support mapping direction argument and of the returned point.
pub trait SupportMap<P: Point, M> {
    // FIXME: add methods that takes a unit `dir` in argument.
    // This might be useful to avoid useless normalizations.
    /// Computes an element of the support function preimage of the object.
    ///
    /// A support function is a function returning the maximum value of the dot product between a
    /// given direction and each point of the shape. The preimage of this function is the set of
    /// points that realize this maximum value. This method returns one of those points picked
    /// arbitrarily.
    ///
    /// # Arguments:
    ///  * `dir` - the input of the support function. It is not required for it to
    ///            be normalized.
    fn support_point(&self, transform: &M, dir: &P::Vect) -> P;

    /// Computes a polyhedral approximation of the preimage of the support support function.
    ///
    /// When several points minimize the dot product on the given direction, this computes a
    /// polyhedral approximation of the vertices of the convex set of points that realize this
    /// minimum.
    ///
    /// Note that this method is currently not implemented for dimensions higher than 3.
    ///
    /// # Arguments:
    ///  * `dir` - the input direction of the support function. It is not required for it to
    ///            be normalized.
    ///  * `eps` - tolerance within which the vertices are considered to realize the same support
    ///            function value. Currently, the interpretation of this "tolerence" is
    ///            implementation-defined.
    ///  * `approx_count` - maximum number of points that may be generated if the point set
    ///                     realizing the support function value is not polyhedral.
    ///  * `out_points` - vector to which the support points are being output.
    ///
    ///  # Returns:
    ///  The number of support points actually output to `out_points`. This might be greater than
    ///  `approx_count` if the point set realizing the support function value is polyhedral and has
    ///  more than `approx_count` vertices.
    fn support_point_set(&self,
                         transform:     &M,
                         dir:           &P::Vect,
                         _eps:          <P::Vect as Vector>::Scalar,
                         _approx_count: usize,
                         out_points:    &mut Vec<P>)
                         -> usize {
        let p = self.support_point(transform, dir);
        out_points.push(p);
        1
     }
}

