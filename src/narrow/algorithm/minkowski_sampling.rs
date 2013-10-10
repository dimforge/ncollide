use std::num::{Zero, One};
use nalgebra::na::{Cast, AlgebraicVecExt, UniformSphereSample, Identity, Translation};
use geom;
use geom::{Implicit, Reflection, MinkowskiSum, AnnotatedPoint};
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<N, AnnotatedPoint<V>>,
                      G1: Implicit<N, V, M>,
                      G2: Implicit<N, V, M>,
                      N:  Clone + Ord + Num + Float + Cast<f32> + ToStr,
                      V:  AlgebraicVecExt<N> + Clone + ToStr,
                      M:  One + Translation<V>>(
                      m1:      &M,
                      g1:      &G1,
                      m2:      &M,
                      g2:      &G2,
                      simplex: &mut S)
                      -> Option<(V, V)> {
    // build the cso with enlarged shapes
    // we enlarge the shapes with a small sphere
    // FIXME: using minkowskiSum(CSO(...)) could be more
    // efficient than the current approach (CSO(minkowskiSum(...), minkowskiSum(..)))
    let _0: N     = Zero::zero();
    let _1m: M    = One::one();
    let reflect2  = Reflection::new(g2);
    let cso       = MinkowskiSum::new(m1, g1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: V = Zero::zero();
    let mut min_dist    = Bounded::max_value();

    do UniformSphereSample::sample() |sample: V| {
        let support = cso.support_point(&Identity::new(), &sample);
        let dist    = sample.dot(&support);

        if (dist < min_dist) {
            best_dir     = sample;
            min_dist     = dist;
        }
    }

    if min_dist <= _0 {
        return None
    }

    let shift = best_dir * min_dist;

    // XXX: translate the simplex instead of reseting it
    let tm2 = m2.translated(&shift.clone());

    simplex.reset(geom::cso_support_point_without_margin(m1, g1, &tm2, g2, best_dir));

    match gjk::closest_points_without_margin(m1, g1, &tm2, g2, simplex) {
        None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            let corrected_normal = (p2 - p1).normalized();

            let corrected_support = cso.support_point(&Identity::new(), &corrected_normal);
            let min_dist2 = corrected_normal.dot(&corrected_support);

            // assert!(min_dist2 >= _0, "Internal error: corrected normal is invalid.");
            if min_dist2 < _0 {
                return None
            }

            let shift2 = corrected_normal * min_dist2;

            Some((p2 - shift + shift2 , p2 - shift))

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
