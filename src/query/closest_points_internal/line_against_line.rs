use crate::math::{Point, Vector};
use na::{self, Real};

/// Closest points between two lines.
///
/// The result, say `res`, is such that the closest points between both lines are
/// `orig1 + dir1 * res.0` and `orig2 + dir2 * res.1`.
#[inline]
pub fn line_against_line_parameters<N: Real>(
    orig1: &Point<N>,
    dir1: &Vector<N>,
    orig2: &Point<N>,
    dir2: &Vector<N>,
) -> (N, N)
{
    let res = line_against_line_parameters_eps(orig1, dir1, orig2, dir2, N::default_epsilon());
    (res.0, res.1)
}

/// Closest points between two lines with a custom tolerance epsilon.
///
/// The result, say `res`, is such that the closest points between both lines are
/// `orig1 + dir1 * res.0` and `orig2 + dir2 * res.1`. If the lines are parallel
/// then `res.2` is set to `true` and the returned closest points are `orig1` and
/// its projection on the second line.
#[inline]
pub fn line_against_line_parameters_eps<N: Real>(
    orig1: &Point<N>,
    dir1: &Vector<N>,
    orig2: &Point<N>,
    dir2: &Vector<N>,
    eps: N,
) -> (N, N, bool)
{
    // Inspired by Real-time collision detection by Christer Ericson.
    let r = *orig1 - *orig2;

    let a = dir1.norm_squared();
    let e = dir2.norm_squared();
    let f = dir2.dot(&r);

    let _0: N = na::zero();
    let _1: N = na::one();


    if a <= eps && e <= eps {
        (_0, _0, false)
    } else if a <= eps {
        (_0, f / e, false)
    } else {
        let c = dir1.dot(&r);
        if e <= eps {
            (-c / a, _0, false)
        } else {
            let b = dir1.dot(dir2);
            let ae = a * e;
            let bb = b * b;
            let denom = ae - bb;

            // Use absolute and ulps error to test collinearity.
            let parallel = denom <= eps || ulps_eq!(ae, bb);

            let s = if !parallel {
                (b * f - c * e) / denom
            } else {
                _0
            };

            (s, (b * s + f) / e, parallel)
        }
    }
}

// FIXME: can we re-used this for the segment/segment case?
/// Closest points between two segments.
#[inline]
pub fn line_against_line<N: Real>(
    orig1: &Point<N>,
    dir1: &Vector<N>,
    orig2: &Point<N>,
    dir2: &Vector<N>,
) -> (Point<N>, Point<N>)
{
    let (s, t) = line_against_line_parameters(orig1, dir1, orig2, dir2);
    (*orig1 + *dir1 * s, *orig2 + *dir2 * t)
}
