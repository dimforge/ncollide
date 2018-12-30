use std::iter::IntoIterator;

use alga::linear::Transformation;
use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector, DIM};
use na::{self, Real};
use crate::shape::SupportMap;

/// Computes the AABB of an support mapped shape.
pub fn support_map_aabb<N, G>(m: &Isometry<N>, i: &G) -> AABB<N>
where
    N: Real,
    G: SupportMap<N>,
{
    let mut min = na::zero::<Vector<N>>();
    let mut max = na::zero::<Vector<N>>();
    let mut basis = na::zero::<Vector<N>>();

    for d in 0..DIM {
        // FIXME: this could be further improved iterating on `m`'s columns, and passing
        // Id as the transformation matrix.
        basis[d] = na::one();
        max[d] = i.support_point(m, &basis)[d];

        basis[d] = -na::one::<N>();
        min[d] = i.support_point(m, &basis)[d];

        basis[d] = na::zero();
    }

    AABB::new(Point::from_coordinates(min), Point::from_coordinates(max))
}

/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<'a, N: Real, M: Transformation<Point<N>>, I>(m: &M, pts: I) -> AABB<N>
    where I: IntoIterator<Item = &'a Point<N>> {
    let mut it = pts.into_iter();

    let p0 = it.next().expect("Point cloud AABB construction: the input iterator should yield at least one point.");
    let wp0 = m.transform_point(&p0);
    let mut min: Point<N> = wp0;
    let mut max: Point<N> = wp0;

    for pt in it {
        let wpt = m.transform_point(pt);
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    AABB::new(min, max)
}
