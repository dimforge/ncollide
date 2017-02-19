use na;
use math::Point;

/// Computes the center of a set of point.
#[inline]
pub fn center<P: Point>(pts: &[P]) -> P {
    assert!(pts.len() >= 1, "Cannot compute the center of less than 1 point.");

    let denom: P::Real = na::convert(1.0 / (pts.len() as f64));

    let mut piter = pts.iter();
    let mut res   = *piter.next().unwrap() * denom;

    for pt in piter {
        res.axpy(denom, pt);
    }

    res
}
