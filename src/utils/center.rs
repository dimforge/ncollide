use na::{Cast, Orig, Axpy};
use na;

/// Computes the center of a set of point.
// FIXME: Dot<N> is here only to determine `N`.
#[inline]
pub fn center<N: Cast<f64>, P: Axpy<N> + Orig>(pts: &[P]) -> P {
    let mut res  = na::orig::<P>();
    let denom: N = na::cast(1.0 / (pts.len() as f64));

    for pt in pts.iter() {
        res.axpy(&denom, pt);
    }

    res
}
