use std::fmt::Show;
use na::{Outer, Inv, Zero};
use na;
use utils;
use math::{Scalar, Point, Vect};

// FIXME: move this to nalgebra?

/// Computes the convariance matrix of a set of points.
pub fn cov<N, P, V, M>(pts: &[P]) -> M
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Outer<M>,
          M: Add<M, M> + Zero {
    cov_and_center(pts).val0()
}

/// Computes the covariance matrix and center of a set of points.
pub fn cov_and_center<N, P, V, M>(pts: &[P]) -> (M, P)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Outer<M>,
          M: Add<M, M> + Zero {
    let center        = utils::center(pts);
    let mut cov: M    = na::zero();
    let normalizer: N = na::cast(1.0 / (pts.len() as f64));

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
pub fn center_reduce<N, P, V, M>(pts: &mut [P]) -> (M, P, bool)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Outer<M>,
          M: Add<M, M> + Zero + Mul<P, P> + Inv + Show {
    let (cov, center) = cov_and_center(pts);

    match na::inv(&cov) {
        None       => (cov, center, false),
        Some(icov) => {
            for pt in pts.iter_mut() {
                // FIXME: the `+ (-` is ugly but required until the trait reform is implemented in
                // rustc!
                *pt = icov * (*pt + (-*center.as_vec()));
            }

            (cov, center, true)
        }
    }
}
