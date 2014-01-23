//! Penetration depth computation algorithm approximating the Minkowskis sum.

use std::num::Bounded;
use nalgebra::na::Identity;
use nalgebra::na;
use geom::{Reflection, MinkowskiSum, AnnotatedPoint};
use implicit::{Implicit, PreferedSamplingDirections};
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;
use math::{N, V, M};

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<AnnotatedPoint>,
                      G1: Implicit<V, M> + PreferedSamplingDirections<V, M>,
                      G2: Implicit<V, M> + PreferedSamplingDirections<V, M>>(
                      m1:      &M,
                      g1:      &G1,
                      m2:      &M,
                      g2:      &G2,
                      simplex: &mut S)
                      -> Option<(V, V)> {
    let _0: N     = na::zero();
    let _1m: M    = na::one();
    let reflect2  = Reflection::new(g2);
    let cso       = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: V = na::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    g1.sample(m1, |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    g2.sample(m2, |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    na::sample_sphere(|sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    if min_dist <= _0 {
        return None
    }

    let shift = best_dir * min_dist;

    let tm2 = na::append_translation(m2, &shift);

    simplex.translate_by(&AnnotatedPoint::new(na::zero(), -shift, -shift));

    match gjk::closest_points_without_margin(m1, g1, &tm2, g2, simplex) {
        None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
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
            let dist_err  = na::norm(&(p2 - p1)) - g1.margin() - g2.margin();
            let _2: N     = na::cast(2.0);
            let center: V = (p1 + p2) / _2;

            let p2 = center - best_dir * if dist_err > na::zero() { min_dist - dist_err } else { min_dist };

            Some((center, p2))
        }
    }
}

#[cfg(dim2, f32, test)]
mod test {
    use super::closest_points;
    use nalgebra::na::{Vec2, Iso2};
    use nalgebra::na;
    use geom::{Box, AnnotatedPoint};
    use implicit;
    use narrow::algorithm::johnson_simplex::JohnsonSimplex;
    use narrow::algorithm::simplex::Simplex;

    #[test]
    fn test_closest_points() {
        let a = Box::new(Vec2::new(5.0f32, 1.0));
        let b = Box::new(Vec2::new(5.0f32, 1.0));
        let ta = Iso2::new(Vec2::new(0.0f32, 0.0), na::zero());
        let tb = Iso2::new(Vec2::new(7.0f32, 1.0), na::zero());
        let mut splx: JohnsonSimplex<AnnotatedPoint> = JohnsonSimplex::new_w_tls();
        let t = Vec2::new(1.0f32, 1.0);
        splx.reset(implicit::cso_support_point_without_margin(&ta, &a, &tb, &b, t));
        println!("{:?}", closest_points(&ta, &a, &tb, &b, &mut splx));
    }
}
