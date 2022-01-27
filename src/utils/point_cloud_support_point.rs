use crate::math::{Point, Vector};
use na::{self, RealField};

/// Computes the index of the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point_id<N: RealField + Copy>(
    dir: &Vector<N>,
    points: &[Point<N>],
) -> usize {
    let mut best_pt = 0;
    let mut best_dot = points[0].coords.dot(dir);

    for i in 1..points.len() {
        let p = &points[i];
        let dot = p.coords.dot(dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt = i;
        }
    }

    best_pt
}

/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<N: RealField + Copy>(
    dir: &Vector<N>,
    points: &[Point<N>],
) -> Point<N> {
    points[point_cloud_support_point_id(dir, points)]
}
