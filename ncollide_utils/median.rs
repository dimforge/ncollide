use alga::general::Real;
use na;

/// Computes the median of a set of values.
#[inline]
pub fn median<N: Real>(vals: &mut [N]) -> N {
    assert!(vals.len() > 0, "Cannot compute the median of zero values.");

    vals.sort_by(|a, b| a.partial_cmp(b).unwrap() );

    let n = vals.len();

    if n % 2 == 0 {
        (vals[n / 2 - 1] + vals[n / 2]) / na::convert(2.0)
    } else {
        vals[n / 2]
    }
}
