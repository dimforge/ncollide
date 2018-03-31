use approx::ApproxEq;
use na::{self, Real};

use math::Point;

// FIXME: can we re-used this for the segment/segment case?
/// Closest points between two segments.
#[inline]
pub fn line_against_line<P: Point>(
    orig1: &P,
    dir1: &P::Vector,
    orig2: &P,
    dir2: &P::Vector
) -> (P, P) {
    // Inspired by Real-time collision detection by Christer Ericson.
    let r = *orig1 - *orig2;

    let a = na::norm_squared(dir1);
    let e = na::norm_squared(dir2);
    let f = na::dot(dir2, &r);

    let _0: P::Real = na::zero();
    let _1: P::Real = na::one();

    let mut s;
    let mut t;

    let _eps = P::Real::default_epsilon();
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
            let denom = ae - b * b;

            // Use relative and absolute error to test collinearity.
            if denom > _eps && denom > _eps * ae {
                s = (b * f - c * e) / denom;
            } else {
                s = _0;
            }

            t = (b * s + f) / e;
        }
    }

    (*orig1 + *dir1 * s, *orig2 + *dir2 * t)
}
