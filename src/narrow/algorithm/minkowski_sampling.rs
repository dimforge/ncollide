use std::num::{Zero, One};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::sample::UniformSphereSample;
use nalgebra::traits::translation::{Translation, Translatable};
use geom::minkowski_sum;
use geom::minkowski_sum::{NonTransformableMinkowskiSum, AnnotatedPoint};
use geom::reflection::Reflection;
use geom::implicit::Implicit;
use geom::ball::Ball;
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;

/// Computes the closest points between two implicit inter-penetrating shapes. Returns None if the
/// shapes are not in penetration. This can be used as a fallback algorithm for the GJK algorithm.
pub fn closest_points<S:  Simplex<N, AnnotatedPoint<V>>,
                      G1: Implicit<V, M>,
                      G2: Implicit<V, M>,
                      N:  Clone + Sub<N, N> + Ord + Mul<N, N> + Float,
                      V:  Norm<N> + Dot<N> + Dim + VectorSpace<N> +
                          UniformSphereSample + Clone,
                      M:  One + Translation<V> + Translatable<V, M>>(
                      m1:      &M,
                      g1:      &G1,
                      m2:      &M,
                      g2:      &G2,
                      margin:  &N,
                      simplex: &mut S)
                      -> Option<(V, V)> {
    // build the cso with enlarged shapes
    // we enlarge the shapes with a small sphere
    // FIXME: using minkowskiSum(CSO(...)) could be more
    // efficient than the current approach (CSO(minkowskiSum(...), minkowskiSum(..)))
    let _0        = Zero::zero::<N>();
    let _1m       = One::one::<M>();
    let enlarger  = Ball::new(margin.clone());
    let enlarged1 = NonTransformableMinkowskiSum::new(m1, g1, &_1m, &enlarger);
    let enlarged2 = NonTransformableMinkowskiSum::new(m2, g2, &_1m, &enlarger);
    let reflect2  = Reflection::new(&enlarged2);
    let cso       = NonTransformableMinkowskiSum::new(m1, &enlarged1, m2, &reflect2);

    // find an approximation of the smallest penetration direction
    let mut best_dir: Option<&'static V> = None;
    let mut min_dist = Bounded::max_value();
    let mut best_support = Zero::zero(); // FIXME: remove that (for debug)

    do UniformSphereSample::sample::<V>() |sample| {
        // NOTE: m1 will be ignored by the minkowski sum
        let support = cso.support_point(m1, sample);
        let dist    = sample.dot(&support);

        if (dist < min_dist) {
            best_dir     = Some(sample);
            best_support = support;
            min_dist     = dist;
        }
    }

    if min_dist <= _0 {
        return None
    }

    let shift = best_dir.unwrap().scalar_mul(&min_dist);

    // XXX: translate the simplex instead of reseting it
    let tm2 = m2.translated(&shift.clone());

    simplex.reset(minkowski_sum::cso_support_point(m1, g1, &tm2, g2, best_dir.unwrap().clone()));

    match gjk::closest_points(m1, g1, &tm2, g2, simplex) {
        None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
        Some((p1, p2)) => {
            let corrected_normal = (p2 - p1).normalized();

            // NOTE: m1 will be ignored by the MinkowskiSum
            let corrected_support = cso.support_point(m1, &corrected_normal);
            let min_dist2 = corrected_normal.dot(&corrected_support);

            // assert!(min_dist2 >= _0, "Internal error: corrected normal is invalid.");
            if min_dist2 < _0 {
                return None
            }

            let shift2 = corrected_normal.scalar_mul(&min_dist2);

            Some((p2 - shift + shift2 , p2 - shift))

            /* This second pass might not really be useful after all
            else {
                // FIXME: (optim) use a scalar_mul_inplace?

                //  FIXME: optimize by translating the last gjk simplex
                match gjk::closest_points::<S, G1, Translated<G2, V>, N, V>
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
