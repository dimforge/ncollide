//! Traits for support mapping based geometries.

use nalgebra::na::FloatVec;
use nalgebra::na;
use math::Scalar;

// Sadly, we cannot put this on the `Implicit` trait because the caller of `margin` might get
// unconstrained type.
/// Trait of geometries having a margin.
pub trait HasMargin {
    /**
     * The geometry margin.
     */
    fn margin(&self) -> Scalar;
}

impl<'a> HasMargin for &'a HasMargin + 'a {
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

/// Traits of convex geometries representable by a support mapping function.
///
/// # Parameters:
///   * Vect - type of the support mapping direction argument and of the returned point.
pub trait Implicit<Vect: FloatVec<Scalar>, Matrix>: HasMargin {
    // FIXME: add methods that takes a unit `dir` in argument.
    // This might be useful to avoid useless normalizations.

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
    fn support_point(&self, transform: &Matrix, dir: &Vect) -> Vect {
        let wo_margin = self.support_point_without_margin(transform, dir);

        wo_margin + na::normalize(dir) * self.margin()
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
    fn support_point_without_margin(&self, transform: &Matrix, dir: &Vect) -> Vect;
}

impl<'a, Vect: FloatVec<Scalar>, Matrix> HasMargin for &'a Implicit<Vect, Matrix> + 'a {
    #[inline]
    fn margin(&self) -> Scalar {
        self.margin()
    }
}

impl<'a, Vect: FloatVec<Scalar>, Matrix> Implicit<Vect, Matrix> for &'a Implicit<Vect, Matrix> + 'a {
    #[inline]
    fn support_point(&self, transform: &Matrix, dir: &Vect) -> Vect {
        self.support_point(transform, dir)
    }

    #[inline]
    fn support_point_without_margin(&self, transform: &Matrix, dir: &Vect) -> Vect {
        self.support_point_without_margin(transform, dir)
    }
}

/// Trait of geometries having prefered sampling directions for the Minkowski sampling algorithm.
///
/// Those directions are usually the geometry faces normals.
pub trait PreferedSamplingDirections<Vect, Matrix> {
    /// Applies a function to this geometry with a given transform.
    fn sample(&self, &Matrix, |Vect| -> ());
}
