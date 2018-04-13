use na;
use math::Point;

/// Computes the index of the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point_id<P: Point>(dir: &P::Vector, points: &[P]) -> usize {
    let mut best_pt = 0;
    let mut best_dot = na::dot(&points[0].coordinates(), dir);

    for i in 1..points.len() {
        let p = &points[i];
        let dot = na::dot(&p.coordinates(), dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt = i;
        }
    }

    best_pt
}

/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<P: Point>(dir: &P::Vector, points: &[P]) -> P {
    points[point_cloud_support_point_id(dir, points)]
}
