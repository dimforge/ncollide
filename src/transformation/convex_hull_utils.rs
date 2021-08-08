use crate::bounding_volume;
use crate::math::Point;
use crate::num::Bounded;
use na::{self, RealField};

/// Returns the index of the support point of a list of points.
pub fn support_point_id<N: RealField + Copy, const D: usize>(
    direction: &na::SVector<N, D>,
    points: &[na::Point<N, D>],
) -> Option<usize> {
    let mut argmax = None;
    let _max: N = Bounded::max_value();
    let mut max = -_max;

    for (id, pt) in points.iter().enumerate() {
        let dot = direction.dot(&pt.coords);

        if dot > max {
            argmax = Some(id);
            max = dot;
        }
    }

    argmax
}

/// Returns the index of the support point of an indexed list of points.
pub fn indexed_support_point_id<N: RealField + Copy, const D: usize>(
    direction: &na::SVector<N, D>,
    points: &[na::Point<N, D>],
    idx: &[usize],
) -> Option<usize> {
    let mut argmax = None;
    let _max: N = Bounded::max_value();
    let mut max = -_max;

    for i in idx.iter() {
        let dot = direction.dot(&points[*i].coords);

        if dot > max {
            argmax = Some(*i);
            max = dot;
        }
    }

    argmax
}

/// Scale and center the given set of point depending on their AABB.
pub fn normalize<N: RealField + Copy>(coords: &mut [Point<N>]) -> (Point<N>, N) {
    let aabb = bounding_volume::local_point_cloud_aabb(&coords[..]);
    let diag = na::distance(&aabb.mins, &aabb.maxs);
    let center = aabb.center();

    for c in coords.iter_mut() {
        *c = (*c + (-center.coords)) / diag;
    }

    (center, diag)
}

/// Scale and translates the given set of point.
pub fn denormalize<N: RealField + Copy>(coords: &mut [Point<N>], center: &Point<N>, diag: N) {
    for c in coords.iter_mut() {
        *c = *c * diag + center.coords;
    }
}
