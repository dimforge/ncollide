use nalgebra::na::{Indexable, Transform, FloatVec, PartialOrd};
use nalgebra::na;
use implicit::Implicit;
use bounding_volume::AABB;
use math::{Scalar, Vect, Matrix};


/// Computes the AABB of an implicit shape.
pub fn implicit_shape_aabb<I: Implicit<Vect, Matrix>>(m: &Matrix, i: &I) -> AABB {
        let mut min:   Vect = na::zero();
        let mut max:   Vect = na::zero();
        let mut basis: Vect = na::zero();

        for d in range(0, na::dim::<Vect>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis.set(d, na::one());
            max.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, -na::one::<Scalar>());
            min.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, na::zero());
        }

        let margin = i.margin();
        AABB::new(min - margin, max + margin)
}

// FIXME: return an AABB?
/// Computes the AABB of a set of point.
pub fn point_cloud_aabb<N, V: PartialOrd + FloatVec<N> + Clone, M: Transform<V>>(m: &M, pts: &[V]) -> (V, V) {
    let wp0        = na::transform(m, &pts[0]);
    let mut min: V = wp0.clone();
    let mut max: V = wp0.clone();

    for pt in pts.slice_from(1).iter() {
        let wpt = na::transform(m, pt);
        min = na::inf(&min, &wpt);
        max = na::sup(&max, &wpt);
    }

    (min, max)
}
