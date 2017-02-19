use num;

use alga::linear::FiniteDimVectorSpace;
use na::Matrix3;
use na;

use math::Point;

/// Computes the volume of a tetrahedron.
#[inline]
pub fn tetrahedron_volume<P: Point>(p1: &P, p2: &P, p3: &P, p4: &P) -> P::Real {
    num::abs(tetrahedron_signed_volume(p1, p2, p3, p4))
}

/// Computes the signed volume of a tetrahedron.
///
/// If it is positive, `p4` is on the half-space pointed by the normal of the oriented triangle
/// `(p1, p2, p3)`.
#[inline]
pub fn tetrahedron_signed_volume<P: Point>(p1: &P, p2: &P, p3: &P, p4: &P) -> P::Real {
    assert!(P::Vector::dimension() == 3);

    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;
    let p1p4 = *p4 - *p1;

    let mat = Matrix3::new(p1p2[0], p1p3[0], p1p4[0],
                           p1p2[1], p1p3[1], p1p4[1],
                           p1p2[2], p1p3[2], p1p4[2]);

    mat.determinant() / na::convert(6.0f64)

}

/// Computes the center of a tetrahedron.
#[inline]
pub fn tetrahedron_center<P: Point>(p1: &P, p2: &P, p3: &P, p4: &P) -> P {
    ::center(&[ *p1, *p2, *p3, *p4 ])
}
