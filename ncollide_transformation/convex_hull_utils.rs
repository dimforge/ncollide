use num::Bounded;

use alga::general::Id;
use na;
use math::Point;
use bounding_volume;

/// Returns the index of the support point of a list of points.
pub fn support_point_id<N: Real>(direction: &Vector<N>, points: &[Point<N>]) -> Option<usize> {
    let mut argmax = None;
    let _max: N = Bounded::max_value();
    let mut max = -_max;

    for (id, pt) in points.iter().enumerate() {
        let dot = na::dot(direction, &pt.coords);

        if dot > max {
            argmax = Some(id);
            max = dot;
        }
    }

    argmax
}

/// Returns the index of the support point of an indexed list of points.
pub fn indexed_support_point_id<N: Real>(
    direction: &Vector<N>,
    points: &[Point<N>],
    idx: &[usize],
) -> Option<usize> {
    let mut argmax = None;
    let _max: N = Bounded::max_value();
    let mut max = -_max;

    for i in idx.iter() {
        let dot = na::dot(direction, &points[*i].coords);

        if dot > max {
            argmax = Some(*i);
            max = dot;
        }
    }

    argmax
}

/// Scale and center the given set of point depending on their AABB.
pub fn normalize<N: Real>(coords: &mut [P]) -> (P, N) {
    let (mins, maxs) = bounding_volume::point_cloud_aabb(&Isometry::identity(), &coords[..]);
    let diag = na::distance(&mins, &maxs);
    let center = na::center(&mins, &maxs);

    for c in coords.iter_mut() {
        *c = (*c + (-center.coords)) / diag;
    }

    (center, diag)
}

/// Scale and translates the given set of point.
pub fn denormalize<N: Real>(coords: &mut [P], center: &P, diag: N) {
    for c in coords.iter_mut() {
        *c = *c * diag + center.coords;
    }
}
