//! Penetration depth computation algorithm approximating the Minkowskis sum.

use num::{Bounded, Zero};

use alga::general::Id;
use alga::linear::{NormedSpace, Translation};
use na::{self, Unit};

use crate::shape::{AnnotatedPoint, MinkowskiSum, Reflection};
use crate::shape::SupportMap;
use crate::query::algorithms::gjk;
use crate::query::algorithms::simplex::Simplex;
use crate::math::{Isometry, Point, Vector};

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<N, S, G1: ?Sized, G2: ?Sized>(
    m1: &Isometry<N>,
    g1: &G1,
    m2: &Isometry<N>,
    g2: &G2,
    simplex: &mut S,
) -> Option<(Point<N>, Point<N>, Unit<Vector<N>>)>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<N>,
    G2: SupportMap<N>,
{
    let reflect2 = Reflection::new(g2);
    let cso = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: Vector<N> = na::zero();
    let mut min_dist = Bounded::max_value();

    Vector<N>::sample_sphere(|sample: Vector<N>| {
        let support = cso.support_point(&Isometry::identity(), &sample);
        let distance = sample.dot(&support.coords);

        if distance < min_dist {
            best_dir = sample;
            min_dist = distance;
        }
    });

    let extra_shift = na::convert(0.01f64); // FIXME: do not hard-code the extra shift?
    let shift = best_dir * (min_dist + extra_shift);

    let tm2 = m2.append_translation(&Isometry<N>::Translation::from(shift).unwrap());

    simplex.modify_pnts(&|pt| pt.translate_2(&(-shift)));

    match gjk::closest_points(m1, g1, &tm2, g2, simplex) {
        None => None, // panic!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            // NOTE: at this point, p1 must *not* be concidered as a good contact point for the
            // first object. For example:
            //
            //
            //                               +-------------+
            //                               |             |
            //                               |    obj2     |
            //                       +-------|-----+       |
            //                       |       +-----+-------+
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            //
            // May Become after shifting:
            //                                      +-------------+
            //                                      |             |
            //                                      |    obj2     |
            //                                      |             |
            //                                p2 -> x-------------+
            //                       +-------------x <- p1
            //                       |             |
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            //
            // Thus, after un-shifting, p1 becomes clearly invalid:
            //
            //                               +-------------+
            //                               |             |
            //                               |    obj2     |
            //                       +-------|-----+ <- p1 |
            //                       | p2 -> +-----+-------+
            //                       |    obj1     |
            //                       |             |
            //                       +-------------+
            let (normal, dist_err) = Unit::new_and_get(p2 - p1);

            if !dist_err.is_zero() {
                let p2 = p2 + (-shift);
                let center = na::center(&p1, &p2);
                let nmin_dist = normal.dot(&best_dir) * (min_dist + extra_shift);

                let p2 = center + (-*normal) * (nmin_dist - dist_err);

                Some((center, p2, normal))
            } else {
                // FIXME: something went wrong here.
                None
            }
        }
    }
}

/// Projects the origin on a support-mapped shape.
///
/// The origin is assumed to be inside of the shape.
pub fn project_origin<N, S, G>(m: &Isometry<N>, g: &G, simplex: &mut S) -> Option<P>
where
    N: Real,
    M: Isometry<P>,
    S: Simplex<N>,
    G: SupportMap<N>,
{
    // find an approximation of the smallest penetration direction
    let mut best_dir: Vector<N> = na::zero();
    let mut min_dist = Bounded::max_value();

    Vector<N>::sample_sphere(|sample: Vector<N>| {
        let support = g.support_point(m, &sample);
        let distance = sample.dot(&support.coords);

        if distance < min_dist {
            best_dir = sample;
            min_dist = distance;
        }
    });

    let extra_shift = na::convert(0.01f64); // FIXME: do not hard-code the extra shift?
    let shift = best_dir * (min_dist + extra_shift);

    let tm = m.append_translation(&Isometry<N>::Translation::from(-shift).unwrap());

    simplex.modify_pnts(&|pt| *pt = *pt + (-shift));

    match gjk::project_origin(&tm, g, simplex) {
        None => None, // panic!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some(p) => {
            let mut normal = -p.coords;
            let dist_err = normal.normalize_mut();

            if !dist_err.is_zero() {
                let nmin_dist = normal.dot(&best_dir) * (min_dist + extra_shift);

                Some(Point::origin() + normal * (nmin_dist - dist_err))
            } else {
                // FIXME: something went wrong here.
                None
            }
        }
    }
}
