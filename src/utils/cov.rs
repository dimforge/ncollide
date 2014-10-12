use std::fmt::Show;
use std::num::Zero;
use na::{Cast, Outer, FloatVec, FloatPntExt, RMul, Inv};
use na;
use utils;

// FIXME: move this to nalgebra?

/// Computes the convariance matrix of a set of points.
pub fn cov<N: Float + Cast<f64>,
           P: FloatPntExt<N, V>,
           V: FloatVec<N> + Outer<M>,
           M: Mul<N, M> + Add<M, M> + Zero>(
           pts: &[P])
        -> M {
    cov_and_center(pts).val0()
}

/// Computes the covariance matrix and center of a set of points.
pub fn cov_and_center<N: Float + Cast<f64>,
                      P: FloatPntExt<N, V>,
                      V: FloatVec<N> + Outer<M>,
                      M: Mul<N, M> + Add<M, M> + Zero>(
                      pts: &[P])
                      -> (M, P) {
    let center        = utils::center(pts);
    let mut cov: M    = na::zero();
    let normalizer: N = na::cast(1.0 / (pts.len() as f64));

    for p in pts.iter() {
        let cp = *p - center;
        cov = cov + na::outer(&cp, &cp) * normalizer;
    }

    (cov, center)
}

/// Centers and reduces a set of data.
///
/// Returns the covariance matrix, the center of the data, and a boolean that is `true` if the
/// operation succeeded (otherwise, the returned value as valid, by the input points are left
/// unchanged).
pub fn center_reduce<N: Float + Cast<f64>,
                     P: FloatPntExt<N, V>,
                     V: FloatVec<N> + Outer<M>,
                     M: Mul<N, M> + Add<M, M> + Zero + RMul<P> + Inv + Show>(
                     pts: &mut [P])
                     -> (M, P, bool) {
    let (cov, center) = cov_and_center(pts);

    match na::inv(&cov) {
        None       => (cov, center, false),
        Some(icov) => {
            for pt in pts.iter_mut() {
                // FIXME: the `+ (-` is ugly but required until the trait reform is implemented in
                // rustc!
                *pt = icov.rmul(&(*pt + (-*center.as_vec())));
            }

            (cov, center, true)
        }
    }
}
