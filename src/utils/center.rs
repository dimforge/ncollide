use nalgebra::na::{FloatVec, Cast};
use nalgebra::na;

/// Computes the center of a set of point.
#[inline]
pub fn center<V: FloatVec<N>, N: Cast<f64>>(pts: &[V]) -> V {
    let mut res  = na::zero::<V>();
    let denom: N = na::cast(1.0 / (pts.len() as f64));

    for pt in pts.iter() {
        res = res + *pt * denom;
    }

    res
}
