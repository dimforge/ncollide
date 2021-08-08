use crate::math::Point;
use na::{self, RealField};

/// Computes the center of a set of point.
#[inline]
pub fn center<N: RealField + Copy>(pts: &[Point<N>]) -> Point<N> {
    assert!(
        pts.len() >= 1,
        "Cannot compute the center of less than 1 point."
    );

    let denom: N = na::convert(1.0 / (pts.len() as f64));

    let mut piter = pts.iter();
    let mut res = *piter.next().unwrap() * denom;

    for pt in piter {
        res += pt.coords * denom;
    }

    res
}
