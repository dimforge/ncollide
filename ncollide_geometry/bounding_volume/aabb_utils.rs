use na;
use shape::SupportMap;
use bounding_volume::AABB;
use math::{Isometry, Point};

/// Computes the AABB of an support mapped shape.
pub fn support_map_aabb<P, M, G>(m: &M, i: &G) -> AABB<P>
where
    P: Point,
    M: Isometry<P>,
    G: SupportMap<P, M>,
{
    let mut min = na::zero::<P::Vector>();
    let mut max = na::zero::<P::Vector>();
    let mut basis = na::zero::<P::Vector>();

    for d in 0..na::dimension::<P::Vector>() {
        // FIXME: this could be further improved iterating on `m`'s columns, and passing
        // Id as the transformation matrix.
        basis[d] = na::one();
        max[d] = i.support_point(m, &basis)[d];

        basis[d] = -na::one::<P::Real>();
        min[d] = i.support_point(m, &basis)[d];

        basis[d] = na::zero();
    }

    AABB::new(P::from_coordinates(min), P::from_coordinates(max))
}

// FIXME: return an AABB?
/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<P: Point, M: Isometry<P>>(m: &M, pts: &[P]) -> (P, P) {
    let wp0 = m.transform_point(&pts[0]);
    let mut min: P = wp0;
    let mut max: P = wp0;

    for pt in pts[1..].iter() {
        let wpt = m.transform_point(pt);
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    (min, max)
}
