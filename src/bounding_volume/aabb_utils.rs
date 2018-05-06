use na::{self, Real};
use shape::SupportMap;
use bounding_volume::AABB;
use math::{Isometry, Point, DIM, Vector};

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

// FIXME: return an AABB?
/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<N: Real>(m: &Isometry<N>, pts: &[Point<N>]) -> (Point<N>, Point<N>) {
    let wp0 = m * pts[0];
    let mut min: Point<N> = wp0;
    let mut max: Point<N> = wp0;

    for pt in pts[1..].iter() {
        let wpt = m * pt;
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    (min, max)
}
