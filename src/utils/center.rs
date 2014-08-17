use std::num::Zero;
use nalgebra::na::{Cast, Dot};
use nalgebra::na;

// FIXME: move this − renamed `mean` − to nalgebra?

/// Computes the center of a set of point.
// FIXME: the `Dot` bound is only to help the compile determine the type of N!
#[inline]
pub fn center<N: Cast<f64>, V: Zero + Mul<N, V> + Add<V, V> + Dot<N>>(pts: &[V]) -> V {
    let mut res  = na::zero::<V>();
    let denom: N = na::cast(1.0 / (pts.len() as f64));

    for pt in pts.iter() {
        res = res + *pt * denom;
    }

    res
}
