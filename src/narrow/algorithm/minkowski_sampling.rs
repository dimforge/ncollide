//! Penetration depth computation algorithm approximating the Minkowskis sum.

use std::num::{Zero, Bounded};
use na::{Identity, Norm};
use na;
use geom::{Reflection, MinkowskiSum, AnnotatedPoint};
use implicit::{Implicit, PreferedSamplingDirections};
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;
use math::{Scalar, Vect, Matrix};

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<AnnotatedPoint>,
                      G1: Implicit<Vect, Matrix> + PreferedSamplingDirections<Vect, Matrix>,
                      G2: Implicit<Vect, Matrix> + PreferedSamplingDirections<Vect, Matrix>>(
                      m1:      &Matrix,
                      g1:      &G1,
                      m2:      &Matrix,
                      g2:      &G2,
                      simplex: &mut S)
                      -> Option<(Vect, Vect, Vect)> {
    let _0: Scalar     = na::zero();
    let _1m: Matrix    = na::one();
    let reflect2  = Reflection::new(g2);
    let cso       = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: Vect = na::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    g1.sample(m1, |sample: Vect| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    g2.sample(m2, |sample: Vect| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if dist < min_dist {
            best_dir = sample;
            min_dist = dist;
        }
    });

    // FIXME: avoid code duplication for the closure
    na::sample_sphere(|sample: Vect| {
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

    let extra_shift = na::cast(0.01f64); // FIXME: do not hard-code the extra shift?
    let shift       = best_dir * (min_dist + extra_shift);

    let tm2 = na::append_translation(m2, &shift);

    simplex.translate_by(&AnnotatedPoint::new(na::zero(), -shift, -shift));

    match gjk::closest_points(m1, g1, &tm2, g2, simplex) {
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
            let mut normal = p2 - p1;
            let dist_err   = normal.normalize();

            if !dist_err.is_zero() {
                let _2: Scalar   = na::cast(2.0f64);
                let p2 = p2 - shift;
                let center: Vect = (p1 + p2) / _2;
                let nmin_dist = na::dot(&normal, &best_dir) * min_dist;

                let p2 = center - normal * if dist_err > na::zero() { nmin_dist - dist_err } else { nmin_dist };

                Some((center, p2, normal))
            }
            else {
                // FIXME: something went wrong here.
                None
            }
        }
    }
}

#[cfg(all(dim2, f32, test))]
mod test {
    use super::closest_points;
    use na::{Vec2, Iso2};
    use na;
    use geom::{Cuboid, AnnotatedPoint};
    use implicit;
    use narrow::algorithm::johnson_simplex::JohnsonSimplex;
    use narrow::algorithm::simplex::Simplex;

    #[test]
    fn test_closest_points() {
        let a = Cuboid::new(Vec2::new(5.0f32, 1.0));
        let b = Cuboid::new(Vec2::new(5.0f32, 1.0));
        let ta = Iso2::new(Vec2::new(0.0f32, 0.0), na::zero());
        let tb = Iso2::new(Vec2::new(7.0f32, 1.0), na::zero());
        let mut splx: JohnsonSimplex<AnnotatedPoint> = JohnsonSimplex::new_w_tls();
        let t = Vec2::new(1.0f32, 1.0);
        splx.reset(implicit::cso_support_point_without_margin(&ta, &a, &tb, &b, t));
        println!("{}", closest_points(&ta, &a, &tb, &b, &mut splx));
    }
}
