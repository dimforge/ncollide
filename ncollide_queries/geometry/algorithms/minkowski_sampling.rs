//! Penetration depth computation algorithm approximating the Minkowskis sum.

use na::{Identity, Norm, UniformSphereSample, Translation, Translate, Zero, Bounded};
use na;
use entities::shape::{MinkowskiSum, AnnotatedPoint, Reflection};
use entities::support_map::{SupportMap, PreferedSamplingDirections};
use geometry::algorithms::gjk;
use geometry::algorithms::simplex::Simplex;
use math::{Scalar, Point, Vect};


/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<N, P, V, M, S, G1: ?Sized, G2: ?Sized>(
                      m1: &M, g1: &G1, m2: &M, g2: &G2, simplex: &mut S) -> Option<(P, P, V)>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Translation<V>,
          S:  Simplex<N, AnnotatedPoint<P>>,
          G1: SupportMap<P, V, M> + PreferedSamplingDirections<V, M>,
          G2: SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {
    let _0: N = na::zero();
    let reflect2   = Reflection::new(g2);
    let cso        = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: V = na::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    g1.sample(m1, &mut |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, support.as_vec());

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    g2.sample(m2, &mut |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, support.as_vec());

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    na::sample_sphere(|sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, support.as_vec());

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    let extra_shift = na::cast(0.01f64); // FIXME: do not hard-code the extra shift?
    let shift       = best_dir * (min_dist + extra_shift);

    let tm2 = na::append_translation(m2, &shift);

    simplex.modify_pnts(&|&: pt| pt.translate_2(&(-shift)));

    match gjk::closest_points(m1, g1, &tm2, g2, simplex) {
        None => None, // panic!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            // NOTE: at this point, p1 must *not* be concidered as a good contact point for the
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
            let mut normal = p2 - p1;
            let dist_err   = normal.normalize_mut();

            if !dist_err.is_zero() {
                let p2        = p2 + (-shift);
                let center    = na::center(&p1, &p2);
                let nmin_dist = na::dot(&normal, &best_dir) * (min_dist + extra_shift);

                let p2 = center + (-normal) * (nmin_dist - dist_err);

                Some((center, p2, normal))
            }
            else {
                // FIXME: something went wrong here.
                None
            }
        }
    }
}

/// Projects the origin on a support-mapped shape.
///
/// The origin is assumed to be inside of the shape.
pub fn project_origin<N, P, V, M, S, G>(m: &M, g: &G, simplex: &mut S) -> Option<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translation<V>,
          S: Simplex<N, P>,
          G: SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {
    // find an approximation of the smallest penetration direction
    let mut best_dir: V = na::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    g.sample(m, &mut |sample: V| {
        let support = g.support_point(m, &sample);
        let dist    = na::dot(&sample, support.as_vec());

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    na::sample_sphere(|sample: V| {
        let support = g.support_point(m, &sample);
        let dist    = na::dot(&sample, support.as_vec());

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    let extra_shift = na::cast(0.01f64); // FIXME: do not hard-code the extra shift?
    let shift       = best_dir * (min_dist + extra_shift);

    let tm = na::append_translation(m, &-shift);

    simplex.modify_pnts(&|&: pt| *pt = *pt + (-shift));

    match gjk::project_origin(&tm, g, simplex) {
        None => None, // panic!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some(p) => {
            let mut normal = -*p.as_vec();
            let dist_err   = normal.normalize_mut();

            if !dist_err.is_zero() {
                let nmin_dist = na::dot(&normal, &best_dir) * (min_dist + extra_shift);

                Some(na::orig::<P>() + normal * (nmin_dist - dist_err))
            }
            else {
                // FIXME: something went wrong here.
                None
            }
        }
    }
}
