use na::{Indexable, Transform, PartialOrd, FloatPnt};
use na;
use implicit::Implicit;
use bounding_volume::AABB;
use math::{Scalar, Point, Vect, Matrix};


/// Computes the AABB of an implicit shape.
pub fn implicit_shape_aabb<I: Implicit<Point, Vect, Matrix>>(m: &Matrix, i: &I) -> AABB {
        let mut min:   Point = na::orig();
        let mut max:   Point = na::orig();
        let mut basis: Vect  = na::zero();

        for d in range(0, na::dim::<Vect>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis[d] = na::one();
            max[d] = i.support_point(m, &basis).at(d);

            basis[d] = -na::one::<Scalar>();
            min[d] = i.support_point(m, &basis).at(d);

            basis[d] = na::zero();
        }

        AABB::new(min, max)
}

// FIXME: return an AABB?
/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<N, P: PartialOrd + FloatPnt<N, V> + Clone, V, M: Transform<P>>(m: &M, pts: &[P]) -> (P, P) {
    let wp0        = na::transform(m, &pts[0]);
    let mut min: P = wp0.clone();
    let mut max: P = wp0.clone();

    for pt in pts.slice_from(1).iter() {
        let wpt = na::transform(m, pt);
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    (min, max)
}
