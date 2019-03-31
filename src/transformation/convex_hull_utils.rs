use alga::linear::EuclideanSpace;
use crate::bounding_volume;
use crate::math::{Isometry, Point};
use na::{self, RealField};
use crate::num::Bounded;

/// Returns the index of the support point of a list of points.
pub fn support_point_id<P: EuclideanSpace>(
    direction: &P::Coordinates,
    points: &[P],
) -> Option<usize>
{
    use alga::linear::FiniteDimVectorSpace;

    let mut argmax = None;
    let _max: P::RealField = Bounded::max_value();
    let mut max = -_max;

    for (id, pt) in points.iter().enumerate() {
        let dot = direction.dot(&pt.coordinates());

        if dot > max {
            argmax = Some(id);
            max = dot;
        }
    }

    argmax
}

/// Returns the index of the support point of an indexed list of points.
pub fn indexed_support_point_id<P: EuclideanSpace>(
    direction: &P::Coordinates,
    points: &[P],
    idx: &[usize],
) -> Option<usize>
{
    use alga::linear::FiniteDimVectorSpace;

    let mut argmax = None;
    let _max: P::RealField = Bounded::max_value();
    let mut max = -_max;

    for i in idx.iter() {
        let dot = direction.dot(&points[*i].coordinates());

        if dot > max {
            argmax = Some(*i);
            max = dot;
        }
    }

    argmax
}

/// Scale and center the given set of point depending on their AABB.
pub fn normalize<N: RealField>(coords: &mut [Point<N>]) -> (Point<N>, N) {
    let aabb = bounding_volume::point_cloud_aabb(&Isometry::identity(), &coords[..]);
    let diag = na::distance(aabb.mins(), aabb.maxs());
    let center = aabb.center();

    for c in coords.iter_mut() {
        *c = (*c + (-center.coords)) / diag;
    }

    (center, diag)
}

/// Scale and translates the given set of point.
pub fn denormalize<N: RealField>(coords: &mut [Point<N>], center: &Point<N>, diag: N) {
    for c in coords.iter_mut() {
        *c = *c * diag + center.coords;
    }
}
