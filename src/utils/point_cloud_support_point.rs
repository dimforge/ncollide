use na::{self, Real};
use math::{Point, Vector};

/// Computes the index of the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point_id<N: Real>(dir: &Vector<N>, points: &[Point<N>]) -> usize {
    let mut best_pt = 0;
    let mut best_dot = na::dot(&points[0].coords, dir);

    for i in 1..points.len() {
        let p = &points[i];
        let dot = na::dot(&p.coords, dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt = i;
        }
    }

    best_pt
}

/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<N: Real>(dir: &Vector<N>, points: &[Point<N>]) -> Point<N> {
    points[point_cloud_support_point_id(dir, points)]
}
