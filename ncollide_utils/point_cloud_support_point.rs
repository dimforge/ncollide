use na;
use math::Point;

/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<P: Point>(dir: &P::Vector, points: &[P]) -> P {
    let mut best_pt  = &points[0];
    let mut best_dot = na::dot(&best_pt.coordinates(), dir);;

    for p in points[1 ..].iter() {
        let dot = na::dot(&p.coordinates(), dir);

        if dot > best_dot {
            best_dot = dot;
            best_pt  = p;
        }
    }

    *best_pt
}
