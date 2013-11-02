use std::num::{Zero, One};
use nalgebra::na::{Cast, AlgebraicVecExt, UniformSphereSample, Identity, Translation};
use nalgebra::na;
use geom;
use geom::{Implicit, Reflection, MinkowskiSum, AnnotatedPoint};
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;

pub trait PreferedSamplingDirections<V, M> {
    fn sample(&self, &M, &fn(V));
}

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<N, AnnotatedPoint<V>>,
                      G1: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
                      G2: Implicit<N, V, M> + PreferedSamplingDirections<V, M>,
                      N:  Clone + Ord + Num + Float + Cast<f32>,
                      V:  AlgebraicVecExt<N> + Clone,
                      M:  One + Translation<V>>(
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
    let mut best_dir: V = Zero::zero();
    let mut min_dist    = Bounded::max_value();

    // FIXME: avoid code duplication for the closure
    do g1.sample(m1) |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    }

    // FIXME: avoid code duplication for the closure
    do g2.sample(m2) |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    }

    // FIXME: avoid code duplication for the closure
    do na::sample_sphere |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = na::dot(&sample, &support);

        if (dist < min_dist) {
            best_dir = sample;
            min_dist = dist;
        }
    }

    if min_dist <= _0 {
        return None
    }

    let shift = best_dir * min_dist;

    let tm2 = na::append_translation(m2, &shift);

    // XXX: translate the simplex instead of reseting it
    simplex.reset(geom::cso_support_point_without_margin(m1, g1, &tm2, g2, best_dir.clone()));

    match gjk::closest_points_without_margin(m1, g1, &tm2, g2, simplex) {
        None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            let corrected_normal = na::normalize(&(p2 - p1));

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

            /*
            let corrected_support = cso.support_point(&Identity::new(), &corrected_normal);
            let min_dist2 = na::dot(&corrected_normal, &corrected_support);

            println!("min_dist: {:?}, min_dist2: {:?}", min_dist, min_dist2);
            println!("p2: {:?}, shift: {:?}, corrected_normal: {:?}", p2, shift, corrected_normal);
            Some((p2 - shift + corrected_normal * (na::dot(&corrected_normal, &shift) - g2.margin()),
                  p2 - shift - corrected_normal * g2.margin()))
                  */

            // let corrected_support = cso.support_point(&Identity::new(), &corrected_normal);
            // let min_dist2 = na::dot(&corrected_normal, &corrected_support);

            // // assert!(min_dist2 >= _0, "Internal error: corrected normal is invalid.");
            // if min_dist2 < _0 {
            //     println!("Error in phase 2");
            //     return None
            // }

            // let shift2 = corrected_normal * min_dist2;

            let dist_err = na::norm(&(p2 - p1)) - g1.margin() - g2.margin();
            let center   = (p1 + p2) / na::cast(2.0);

            let p2 = center - best_dir * if dist_err > na::zero() { min_dist - dist_err } else { min_dist };

            Some((center, p2))

            /* This second pass might not really be useful after all
            else {
                // FIXME: (optim) use a scalar_mul_inplace?

                //  FIXME: optimize by translating the last gjk simplex
                match gjk::closest_points_without_margin::<S, G1, Translated<G2, V>, N, V>
                (g1, &Translated::new(g2, shift2.clone())) {
                    None =>
                    fail!("Internal error: the origin was inside of the Simplex during phase 2."),
                    Some((res1, res2)) => Some((res1, res2 - shift2))
                }
            }
            */

        }
    }
}

#[cfg(test)]
mod test {
    use super::closest_points;
    use nalgebra::na::Vec2;
    use geom::{Box, AnnotatedPoint};
    use narrow::algorithm::johnson_simplex::JohnsonSimplex;

    #[test]
    fn test_closest_points() {
        let a = Box::new(Vec2::new(5.0f32, 1.0));
        let b = Box::new(Vec2::new(5.0f32, 1.0));
        let ta = Vec2::new(0.0f32, 0.0);
        let tb = Vec2::new(7.0f32, 1.0);
        let mut splx: JohnsonSimplex<f32, AnnotatedPoint<Vec2<f32>>> = JohnsonSimplex::new_w_tls();
        println!("{:?}", closest_points(&ta, &a, &tb, &b, &mut splx));
    }
}
