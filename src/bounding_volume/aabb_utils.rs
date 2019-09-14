use std::iter::IntoIterator;

use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::shape::SupportMap;
use alga::linear::Transformation;
use na::{self, RealField};

/// Computes the AABB of an support mapped shape.
pub fn support_map_aabb<N, G>(m: &Isometry<N>, i: &G) -> AABB<N>
where
    N: RealField,
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

    AABB::new(Point::from(min), Point::from(max))
}

/// Computes the AABB of a set of points transformed by `m`.
pub fn point_cloud_aabb<'a, N: RealField, M: Transformation<Point<N>>, I>(m: &M, pts: I) -> AABB<N>
where
    I: IntoIterator<Item = &'a Point<N>>,
{
    let mut it = pts.into_iter();

    let p0 = it.next().expect(
        "Point cloud AABB construction: the input iterator should yield at least one point.",
    );
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

/// Computes the AABB of a set of points.
pub fn local_point_cloud_aabb<'a, N: RealField, I>(pts: I) -> AABB<N>
where
    I: IntoIterator<Item = &'a Point<N>>,
{
    let mut it = pts.into_iter();

    let p0 = it.next().expect(
        "Point cloud AABB construction: the input iterator should yield at least one point.",
    );
    let mut min: Point<N> = *p0;
    let mut max: Point<N> = *p0;

    for pt in it {
        min = na::inf(&min, &pt);
        max = na::sup(&max, &pt);
    }

    AABB::new(min, max)
}
