use na::{Mat3, Axpy, ScalarMul};
use na;
use utils;
use math::{Scalar, Point};

/// Computes the volume of a tetrahedron.
#[inline]
pub fn tetrahedron_volume<N, P, V>(p1: &P, p2: &P, p3: &P, p4: &P) -> N
    where N: Scalar,
          P: Point<N, V>,
          V: Index<uint, N> {
    tetrahedron_signed_volume(p1, p2, p3, p4).abs()
}

/// Computes the signed volume of a tetrahedron.
///
/// If it is positive, `p4` is on the half-space pointed by the normal of the oriented triangle
/// `(p1, p2, p3)`.
#[inline]
pub fn tetrahedron_signed_volume<N, P, V>(p1: &P, p2: &P, p3: &P, p4: &P) -> N
    where N: Scalar,
          P: Point<N, V>,
          V: Index<uint, N> {
    assert!(na::dim::<P>() == 3);

    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;
    let p1p4 = *p4 - *p1;

    let mat = Mat3::new(p1p2[0], p1p3[0], p1p4[0],
                        p1p2[1], p1p3[1], p1p4[1],
                        p1p2[2], p1p3[2], p1p4[2]);

    na::det(&mat) / na::cast(6.0f64)

}

/// Computes the center of a tetrahedron.
#[inline]
// FIXME: those bounds should be simplified once the trait reform lands:
// N where P: FloatPnt<f64>
pub fn tetrahedron_center<N, P>(p1: &P, p2: &P, p3: &P, p4: &P) -> P
    where N: Scalar,
          P: Axpy<N> + ScalarMul<N> + Clone {
    utils::center(&[ p1.clone(), p2.clone(), p3.clone(), p4.clone() ])
}
