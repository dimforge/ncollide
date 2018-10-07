use math::{Point, Vector};
use na::{self, Real};


/// Closest points between two segments.
///
/// The result, say `res`, is such that the closest points between both lines are
/// `orig1 + dir1 * res.0` and `orig2 + dir2 * res.1`.
#[inline]
pub fn line_against_line_parameters<N: Real>(
    orig1: &Point<N>,
    dir1: &Vector<N>,
    orig2: &Point<N>,
    dir2: &Vector<N>,
) -> (N, N) {
    // Inspired by Real-time collision detection by Christer Ericson.
    let r = *orig1 - *orig2;

    let a = na::norm_squared(dir1);
    let e = na::norm_squared(dir2);
    let f = na::dot(dir2, &r);

    let _0: N = na::zero();
    let _1: N = na::one();

    let s;
    let t;

    let _eps = N::default_epsilon();
    if a <= _eps && e <= _eps {
        s = _0;
        t = _0;
    } else if a <= _eps {
        s = _0;
        t = f / e;
    } else {
        let c = na::dot(dir1, &r);
        if e <= _eps {
            t = _0;
            s = -c / a;
        } else {
            let b = na::dot(dir1, dir2);
            let ae = a * e;
            let bb = b * b;
            let denom = ae - bb;

            // Use absolute and ulps error to test collinearity.
            if denom > _eps && !ulps_eq!(ae, bb) {
                s = (b * f - c * e) / denom;
            } else {
                s = _0;
            }

            t = (b * s + f) / e;
        }
    }

    (s, t)
}

// FIXME: can we re-used this for the segment/segment case?
/// Closest points between two segments.
#[inline]
pub fn line_against_line<N: Real>(
    orig1: &Point<N>,
    dir1: &Vector<N>,
    orig2: &Point<N>,
    dir2: &Vector<N>,
) -> (Point<N>, Point<N>) {
    let (s, t) = line_against_line_parameters(orig1, dir1, orig2, dir2);
    (*orig1 + *dir1 * s, *orig2 + *dir2 * t)
}
