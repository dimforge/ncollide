use std::num::Bounded;
use nalgebra::na::FloatVec;
use nalgebra::na;

/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<V: FloatVec<N> + Clone, N: Float>(dir: &V, points: &[V]) -> V {
    let _max: N = Bounded::max_value();
    let mut best_dot = -_max;
    let mut best_pt  = &points[0];

    for p in points.iter() {
        let dot = na::dot(p, dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt  = p;
        }
    }

    best_pt.clone()
}
