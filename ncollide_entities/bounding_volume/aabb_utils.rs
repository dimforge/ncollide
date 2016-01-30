use na::Transform;
use na;
use support_map::SupportMap;
use bounding_volume::AABB;
use math::{Point, Vect};



/// Computes the AABB of an support mapped shape.
pub fn implicit_shape_aabb<P, M, G>(m: &M, i: &G) -> AABB<P>
        where P: Point,
              G: SupportMap<P, M> {
        let mut min   = na::orig::<P>();
        let mut max   = na::orig::<P>();
        let mut basis = na::zero::<P::Vect>();

        for d in 0 .. na::dim::<P::Vect>() {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis[d] = na::one();
            max[d] = i.support_point(m, &basis)[d];

            basis[d] = -na::one::<<P::Vect as Vect>::Scalar>();
            min[d] = i.support_point(m, &basis)[d];

            basis[d] = na::zero();
        }

        AABB::new(min, max)
}

// FIXME: return an AABB?
/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<P, M>(m: &M, pts: &[P]) -> (P, P)
    where P: Point,
          M: Transform<P> {
    let wp0        = na::transform(m, &pts[0]);
    let mut min: P = wp0.clone();
    let mut max: P = wp0.clone();

    for pt in pts[1 ..].iter() {
        let wpt = na::transform(m, pt);
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    (min, max)
}
