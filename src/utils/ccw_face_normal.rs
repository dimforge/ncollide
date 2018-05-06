
use na::{Real, Unit};
use math::{Vector, Point};

/// Computes the direction pointing toward the right-hand-side of an oriented segment.
/// 
/// Returns `None` if the segment is degenerate.
#[inline]
#[cfg(feature = "dim2")]
pub fn ccw_face_normal<N: Real>(pts: [&Point<N>; 2]) -> Option<Unit<Vector<N>>> {
    let ab = pts[1] - pts[0];
    let res = Vector::new(ab[1], -ab[0]);

    Unit::try_new(res, N::default_epsilon())
}

/// Computes the normal of a counter-clock-wise triangle.
/// 
/// Returns `None` if the triangle is degenerate.
#[inline]
#[cfg(feature = "dim3")]
pub fn ccw_face_normal<N: Real>(pts: [&Point<N>; 3]) -> Option<Unit<Vector<N>>> {
    let ab = pts[1] - pts[0];
    let ac = pts[2] - pts[0];
    let res = ab.cross(&ac);

    Unit::try_new(res, N::default_epsilon())
}
