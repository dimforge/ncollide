use nalgebra::na::Indexable;
use nalgebra::na;
use implicit::Implicit;
use bounding_volume::AABB;
use math::{Scalar, Vect, Matrix};


/// Computes the AABB of an implicit shape.
pub fn implicit_shape_aabb<I: Implicit<Vect, Matrix>>(m: &Matrix, i: &I) -> AABB {
        let mut resm:  Vect = na::zero();
        let mut resM:  Vect = na::zero();
        let mut basis: Vect = na::zero();

        for d in range(0, na::dim::<Vect>()) {
            // FIXME: this could be further improved iterating on `m`'s columns, and passing
            // Identity as the transformation matrix.
            basis.set(d, na::one());
            resM.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, -na::one::<Scalar>());
            resm.set(d, i.support_point_without_margin(m, &basis).at(d));

            basis.set(d, na::zero());
        }

        let margin = i.margin();
        AABB::new(resm - margin, resM + margin)
}

/// Computes the AABB of a set of point.
pub fn point_cloud_aabb(m: &Matrix, pts: &[Vect]) -> AABB {
    let wp0            = na::transform(m, &pts[0]);
    let mut resm: Vect = wp0.clone();
    let mut resM: Vect = wp0.clone();

    for pt in pts.slice_from(1).iter() {
        let wpt = na::transform(m, pt);
        resm = na::inf(&resm, &wpt);
        resM = na::sup(&resM, &wpt);
    }

    AABB::new(resm, resM)
}
