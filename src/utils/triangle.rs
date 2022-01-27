use crate::math::Point;
use crate::utils;
use na;
use simba::scalar::RealField;

/// Computes the area of a triangle.
#[inline]
pub fn triangle_area<N: RealField + Copy>(pa: &Point<N>, pb: &Point<N>, pc: &Point<N>) -> N {
    // Kahan's formula.
    let mut a = na::distance(pa, pb);
    let mut b = na::distance(pb, pc);
    let mut c = na::distance(pc, pa);

    let (c, b, a) = utils::sort3(&mut a, &mut b, &mut c);
    let a = *a;
    let b = *b;
    let c = *c;

    let sqr = (a + (b + c)) * (c - (a - b)) * (c + (a - b)) * (a + (b - c));

    sqr.sqrt() * na::convert(0.25)
}

/// Computes the center of a triangle.
#[inline]
pub fn triangle_center<N: RealField + Copy>(
    pa: &Point<N>,
    pb: &Point<N>,
    pc: &Point<N>,
) -> Point<N> {
    utils::center(&[*pa, *pb, *pc])
}

/// Computes the perimeter of a triangle.
#[inline]
pub fn triangle_perimeter<N: RealField + Copy>(pa: &Point<N>, pb: &Point<N>, pc: &Point<N>) -> N {
    na::distance(pa, pb) + na::distance(pb, pc) + na::distance(pc, pa)
}

/// Computes the circumcircle of a triangle.
pub fn circumcircle<N: RealField + Copy>(
    pa: &Point<N>,
    pb: &Point<N>,
    pc: &Point<N>,
) -> (Point<N>, N) {
    let a = *pa - *pc;
    let b = *pb - *pc;

    let na = a.norm_squared();
    let nb = b.norm_squared();

    let dab = a.dot(&b);

    let _2: N = na::convert(2.0);
    let denom = _2 * (na * nb - dab * dab);

    if denom.is_zero() {
        // The triangle is degenerate (the three points are colinear).
        // So we find the longest segment and take its center.
        let c = *pa - *pb;
        let nc = c.norm_squared();

        if nc >= na && nc >= nb {
            // Longest segment: [pa, pb]
            (na::center(pa, pb), nc.sqrt() / na::convert(2.0f64))
        } else if na >= nb && na >= nc {
            // Longest segment: [pa, pc]
            (na::center(pa, pc), na.sqrt() / na::convert(2.0f64))
        } else {
            // Longest segment: [pb, pc]
            (na::center(pb, pc), nb.sqrt() / na::convert(2.0f64))
        }
    } else {
        let k = b * na - a * nb;

        let center = *pc + (a * k.dot(&b) - b * k.dot(&a)) / denom;
        let radius = na::distance(pa, &center);

        (center, radius)
    }
}

/// Tests if three 3D points are approximately aligned.
#[cfg(feature = "dim3")]
pub fn is_affinely_dependent_triangle<N: RealField + Copy>(
    p1: &Point<N>,
    p2: &Point<N>,
    p3: &Point<N>,
) -> bool {
    let p1p2 = *p2 - *p1;
    let p1p3 = *p3 - *p1;

    // FIXME: use this as nalgebra standard epsilon?
    let _eps = N::default_epsilon(); // FIXME: use Float::epsilon instead?
    let _eps_tol = _eps * na::convert(100.0f64);

    relative_eq!(
        p1p2.cross(&p1p3).norm_squared(),
        na::zero(),
        epsilon = _eps_tol * _eps_tol
    )
}

/// Tests if a point is inside of a triangle.
pub fn is_point_in_triangle<N: RealField + Copy>(
    p: &Point<N>,
    p1: &Point<N>,
    p2: &Point<N>,
    p3: &Point<N>,
) -> bool {
    let p1p2 = *p2 - *p1;
    let p2p3 = *p3 - *p2;
    let p3p1 = *p1 - *p3;

    let p1p = *p - *p1;
    let p2p = *p - *p2;
    let p3p = *p - *p3;

    let d11 = p1p.dot(&p1p2);
    let d12 = p2p.dot(&p2p3);
    let d13 = p3p.dot(&p3p1);

    d11 >= na::zero()
        && d11 <= p1p2.norm_squared()
        && d12 >= na::zero()
        && d12 <= p2p3.norm_squared()
        && d13 >= na::zero()
        && d13 <= p3p1.norm_squared()
}

#[cfg(feature = "dim3")]
#[cfg(test)]
mod test {
    use na::Point3;

    #[test]
    fn test_triangle_area() {
        let pa = Point3::new(0.0f64, 5.0, 0.0);
        let pb = Point3::new(0.0f64, 0.0, 0.0);
        let pc = Point3::new(0.0f64, 0.0, 4.0);

        assert!(relative_eq!(super::triangle_area(&pa, &pb, &pc), 10.0));
    }
}
