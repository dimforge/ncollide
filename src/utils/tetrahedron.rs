use nalgebra::na::{Vec3, Mat3, Cast};
use nalgebra::na::overload::Vec3MulRhs;
use nalgebra::na;
use utils;

/// Computes the volume of a tetrahedron.
#[inline]
pub fn tetrahedron_volume<N: Float + Cast<f64>>(p1: &Vec3<N>, p2: &Vec3<N>, p3: &Vec3<N>, p4: &Vec3<N>) -> N {
    tetrahedron_signed_volume(p1, p2, p3, p4).abs()
}

/// Computes the signed volume of a tetrahedron.
///
/// If it is positive, `p4` is on the half-space pointed by the normal of the oriented triangle
/// `(p1, p2, p3)`.
#[inline]
pub fn tetrahedron_signed_volume<N: Float + Cast<f64>>(p1: &Vec3<N>, p2: &Vec3<N>, p3: &Vec3<N>, p4: &Vec3<N>) -> N {
    let p1p2 = p2 - *p1;
    let p1p3 = p3 - *p1;
    let p1p4 = p4 - *p1;

    let mat = Mat3::new(p1p2.x, p1p3.x, p1p4.x,
                        p1p2.y, p1p3.y, p1p4.y,
                        p1p2.z, p1p3.z, p1p4.z);

    na::det(&mat) / na::cast(6.0f64)

}

/// Computes the center of a tetrahedron.
#[inline]
// FIXME: those bounds should be simplified once the trait reform lands:
// N where Vec3<N>: FloatVec<f64>
pub fn tetrahedron_center<N: Float + Cast<f64> + Vec3MulRhs<N, Vec3<N>>>(
                          p1: &Vec3<N>, p2: &Vec3<N>, p3: &Vec3<N>, p4: &Vec3<N>) -> Vec3<N> {
    utils::center(&[ p1.clone(), p2.clone(), p3.clone(), p4.clone() ])
}
