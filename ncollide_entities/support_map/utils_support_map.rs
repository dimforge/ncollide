use na::Bounded;
use na;
use math::{Scalar, Point, Vect};

// FIXME: move this to `utils::`?
/// Computes the support point of a cloud of points.
#[inline]
pub fn point_cloud_support_point<P>(dir: &P::Vect, points: &[P]) -> P
    where P: Point {
    let _max: <P::Vect as Vect>::Scalar = Bounded::max_value();
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
