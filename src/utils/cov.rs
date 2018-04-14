use std::ops::Mul;
use std::fmt::Debug;
use num::Zero;
use na::{Outer, Inverse};
use na;
use math::{Point, Vector};

// FIXME: move this to nalgebra?

/// Computes the convariance matrix of a set of points.
pub fn cov<P>(pts: &[Point<N>]) -> <Vector<N> as Outer>::OuterProductType
    where N: Real,
          Vector<N>: Outer,
          <Vector<N> as Outer>::OuterProductType: Zero {
    cov_and_center(pts).0
}

/// Computes the covariance matrix and center of a set of points.
pub fn cov_and_center<P>(pts: &[Point<N>]) -> (<Vector<N> as Outer>::OuterProductType, P)
    where N: Real,
          Vector<N>: Outer,
          <Vector<N> as Outer>::OuterProductType: Zero {
    let center = ::center(pts);
    let mut cov: <Vector<N> as Outer>::OuterProductType = na::zero();
    let normalizer: N = na::convert(1.0 / (pts.len() as f64));

    for p in pts.iter() {
        let cp = *p - center;
        cov = cov + na::outer(&cp, &(cp * normalizer));
    }

    (cov, center)
}

/// Centers and reduces a set of data.
///
/// Returns the covariance matrix, the center of the data, and a boolean that is `true` if the
/// operation succeeded (otherwise, the returned value as valid, by the input points are left
/// unchanged).
pub fn center_reduce<P>(pts: &mut [P]) -> (<Vector<N> as Outer>::OuterProductType, P, bool)
    where N: Real,
          Vector<N>: Vector + Outer,
          <Vector<N> as Outer>::OuterProductType: Zero + Mul<P, Output = P> + Inverse + Copy + Debug {
    let (cov, center) = cov_and_center(pts);

    match na::inverse(&cov) {
        None       => (cov, center, false),
        Some(icov) => {
            for pt in pts.iter_mut() {
                // FIXME: the `+ (-` is ugly but required until the trait reform is implemented in
                // rustc!
                *pt = icov * (*pt + (-*center.as_vector()));
            }

            (cov, center, true)
        }
    }
}
