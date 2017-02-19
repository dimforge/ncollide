use num::Bounded;

use alga::general::Id;
use na;
use math::Point;
use geometry::bounding_volume;

/// Returns the index of the support point of a list of points.
pub fn support_point_id<P: Point>(direction: &P::Vector, points : &[P]) -> Option<usize> {
    let mut argmax = None;
    let _max: P::Real = Bounded::max_value();
    let mut max    = -_max;

    for (id, pt) in points.iter().enumerate() {
        let dot = na::dot(direction, &pt.coordinates());

        if dot > max {
            argmax = Some(id);
            max    = dot;
        }
    }

    argmax
}

/// Returns the index of the support point of an indexed list of points.
pub fn indexed_support_point_id<P: Point>(direction: &P::Vector, points : &[P], idx: &[usize]) -> Option<usize> {
    let mut argmax = None;
    let _max: P::Real = Bounded::max_value();
    let mut max    = -_max;

    for i in idx.iter() {
        let dot = na::dot(direction, &points[*i].coordinates());

        if dot > max {
            argmax = Some(*i);
            max    = dot;
        }
    }

    argmax
}

/// Scale and center the given set of point depending on their AABB.
pub fn normalize<P: Point>(coords: &mut [P]) -> (P, P::Real) {
    let (mins, maxs) = bounding_volume::point_cloud_aabb(&Id::new(), &coords[..]);
    let diag   = na::distance(&mins, &maxs);
    let center = na::center(&mins, &maxs);

    for c in coords.iter_mut() {
        *c = (*c + (-center.coordinates())) / diag;
    }

    (center, diag)
}


/// Scale and translates the given set of point.
pub fn denormalize<P: Point>(coords: &mut [P], center: &P, diag: P::Real) {
    for c in coords.iter_mut() {
        *c = *c * diag + center.coordinates();
    }
}
