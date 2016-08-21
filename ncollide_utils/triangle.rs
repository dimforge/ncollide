use std::ops::{Sub, Mul, Index, IndexMut};
use num::Zero;
use na::{Cross, Norm, Dimension, Axpy};
use na;
use num::Float;
use math::{Scalar, Point, Vector, FloatError};

/// Computes the area of a triangle.
#[inline]
pub fn triangle_area<P>(pa: &P, pb: &P, pc: &P) -> <P::Vect as Vector>::Scalar
    where P: Point {
    // Kahan's formula.
    let mut a = na::distance(pa, pb);
    let mut b = na::distance(pb, pc);
    let mut c = na::distance(pc, pa);

    let (c, b, a) = ::sort3(&mut a, &mut b, &mut c);
    let a = *a;
    let b = *b;
    let c = *c;

    let sqr = (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c));

    sqr.sqrt() * na::cast(0.25)
}

/// Computes the center of a triangle.
#[inline]
pub fn triangle_center<N, P>(pa: &P, pb: &P, pc: &P) -> P
    where N: Scalar,
          P: Axpy<N> + Mul<N, Output = P> + Copy {
    ::center(&[ *pa, *pb, *pc ])
}

/// Computes the perimeter of a triangle.
#[inline]
pub fn triangle_perimeter<P>(pa: &P, pb: &P, pc: &P) -> <P::Vect as Vector>::Scalar
    where P: Point {
    na::distance(pa, pb) + na::distance(pb, pc) + na::distance(pc, pa)
}

/// Computes the circumcircle of a triangle.
pub fn circumcircle<P>(pa: &P, pb: &P, pc: &P) -> (P, <P::Vect as Vector>::Scalar)
    where P: Point {
    let a = *pa - *pc;
    let b = *pb - *pc;

    let na = na::norm_squared(&a);
    let nb = na::norm_squared(&b);

    let dab = na::dot(&a, &b);

    let _2: <P::Vect as Vector>::Scalar = na::cast(2.0);
    let denom = _2 * (na * nb - dab * dab);

    if denom.is_zero() {
        // The triangle is degenerate (the three points are colinear).
        // So we find the longest segment and take its center.
        let c = *pa - *pb;
        let nc = na::norm_squared(&c);

        if nc >= na && nc >= nb {
            // Longest segment: [pa, pb]
            (na::center(pa, pb), nc.sqrt() / na::cast(2.0f64))
        }
        else if na >= nb && na >= nc {
            // Longest segment: [pa, pc]
            (na::center(pa, pc), na.sqrt() / na::cast(2.0f64))
        }
        else {
            // Longest segment: [pb, pc]
            (na::center(pb, pc), nb.sqrt() / na::cast(2.0f64))
        }
    }
    else {
        let k = b * na - a * nb;

        let center = *pc + (a * na::dot(&k, &b) - b * na::dot(&k, &a)) / denom;
        let radius = na::distance(pa, &center);

        (center, radius)
    }
}

/// Tests if three 3D points are exactly aligned without the need of the `Cross` trait.
pub fn is_affinely_dependent_triangle3<N, P, V>(p1: &P, p2: &P, p3: &P) -> bool
    where N: Scalar,
          P: Sub<P, Output = V> + Copy,
          V: Zero + Norm<NormType = N> + Index<usize, Output = N> +
             IndexMut<usize, Output = N> + Dimension + Copy {
    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;

    // FIXME: use this as nalgebra standard epsilon?
    let _eps: N = FloatError::epsilon(); // FIXME: use Float::epsilon instead?
    let _eps_tol = _eps * na::cast(100.0f64);

    na::approx_eq_eps(&na::norm_squared(&::cross3(&p1p2, &p1p3)), &na::zero(), &(_eps_tol * _eps_tol))
}

/// Tests if three points are exactly aligned.
pub fn is_affinely_dependent_triangle<N, P, V, AV>(p1: &P, p2: &P, p3: &P) -> bool
    where N:  Scalar,
          P:  Sub<P, Output = V> + Copy,
          V:  Cross<CrossProductType = AV> + Norm<NormType = N> + Copy,
          AV: Norm<NormType = N> {
    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;

    // FIXME: use this as nalgebra standard epsilon?
    let _eps: N  = FloatError::epsilon(); // FIXME: use Float::epsilon instead?
    let _eps_tol = _eps * na::cast(100.0f64);

    na::approx_eq_eps(&na::norm_squared(&na::cross(&p1p2, &p1p3)), &na::zero(), &(_eps_tol * _eps_tol))
}

/// Tests if a point is inside of a triangle.
pub fn is_point_in_triangle<P, V>(p: &P, p1: &P, p2: &P, p3: &P) -> bool
    where P: Sub<P, Output = V> + Copy,
          V: Vector {
    let p1p2 = *p2 - *p1;
    let p2p3 = *p3 - *p2;
    let p3p1 = *p1 - *p3;

    let p1p = *p - *p1;
    let p2p = *p - *p2;
    let p3p = *p - *p3;

    let d11 = na::dot(&p1p, &p1p2);
    let d12 = na::dot(&p2p, &p2p3);
    let d13 = na::dot(&p3p, &p3p1);

    d11 >= na::zero() && d11 <= na::norm_squared(&p1p2) &&
    d12 >= na::zero() && d12 <= na::norm_squared(&p2p3) &&
    d13 >= na::zero() && d13 <= na::norm_squared(&p3p1)
}

#[cfg(test)]
mod test {
    use na;
    use na::Point3;

    #[test]
    fn test_triangle_area() {
        let pa = Point3::new(0.0f64, 5.0, 0.0);
        let pb = Point3::new(0.0f64, 0.0, 0.0);
        let pc = Point3::new(0.0f64, 0.0, 4.0);

        assert!(na::approx_eq(&super::triangle_area(&pa, &pb, &pc), &10.0));
    }
}
