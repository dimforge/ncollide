use std::num::Bounded;
use na::Dot;
use na;
use math::{Scalar, Point};

// FIXME: move this to `utils::`?
/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<N, P, V>(dir: &V, points: &[P]) -> P
    where N: Scalar,
          P: Point<N, V>,
          V: Dot<N> {
    let _max: N = Bounded::max_value();
    let mut best_dot = -_max;
    let mut best_pt  = &points[0];

    for p in points.iter() {
        let dot = na::dot(p.as_vec(), dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt  = p;
        }
    }

    best_pt.clone()
}
