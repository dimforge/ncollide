//! Traits for support mapping based geometries.

use na::FloatVec;
use na;
use math::Scalar;

/// Traits of convex geometries representable by a support mapping function.
///
/// # Parameters:
///   * Vect - type of the support mapping direction argument and of the returned point.
pub trait Implicit<Vect: FloatVec<Scalar>, Matrix> {
    // FIXME: add methods that takes a unit `dir` in argument.
    // This might be useful to avoid useless normalizations.
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
    fn support_point(&self, transform: &Matrix, dir: &Vect) -> Vect;
}

impl<'a, Vect: FloatVec<Scalar>, Matrix> Implicit<Vect, Matrix> for &'a Implicit<Vect, Matrix> + 'a {
    #[inline]
    fn support_point(&self, transform: &Matrix, dir: &Vect) -> Vect {
        self.support_point(transform, dir)
    }
}

/// Trait of geometries having prefered sampling directions for the Minkowski sampling algorithm.
///
/// Those directions are usually the geometry faces normals.
pub trait PreferedSamplingDirections<Vect, Matrix> {
    /// Applies a function to this geometry with a given transform.
    fn sample(&self, &Matrix, |Vect| -> ());
}
