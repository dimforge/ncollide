use std::num::Zero;
use std::rand::Rand;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::vector_space::VectorSpace;
use geom::minkowski_sum::{MinkowskiSum, AnnotatedPoint};
use geom::reflection::Reflection;
use geom::implicit::Implicit;
use geom::ball::Ball;
use geom::translated::Translated;
use narrow::algorithm::gjk;
use narrow::algorithm::simplex::Simplex;

pub fn closest_points<S:  Simplex<AnnotatedPoint<V>, N>,
                      G1: Implicit<V>,
                      G2: Implicit<V>,
                      V:  Norm<N> + Dot<N> + Dim + Rand + VectorSpace<N> + Copy,
                      N:  Sub<N, N> + Ord + Mul<N, N> + Float>
       (g1: &G1, g2: &G2, margin: N) -> Option<(V, V)>
{
  // build the cso with enlarged shapes
  // we enlarge the shapes with a small sphere
  // FIXME: using minkowskiSum(CSO(...)) could be more
  // efficient than the current approach (CSO(minkowskiSum(...), minkowskiSum(..)))
  let _0        = Zero::zero::<N>();
  let enlarger  = Ball::new(Zero::zero::<V>(), margin);
  let enlarged1 = MinkowskiSum::new(g1, &enlarger);
  let enlarged2 = MinkowskiSum::new(g2, &enlarger);
  let reflect2  = Reflection::new(&enlarged2);
  let cso       = MinkowskiSum::new(&enlarged1, &reflect2);

  // find an approximation of the smallest penetration direction
  let mut best_dir_id = 0;
  let mut min_dist    = Bounded::max_value();

  let fixme: ~[V] = ~[]; // FIXME (const V& dir : UnitSphereSamples<V, numSamples>::g1mples())

  for fixme.iter().enumerate().advance |(i, v)|
  {
    let dist = v.dot(&cso.support_point(v));

    if (dist < min_dist)
    {
      best_dir_id = i;
      min_dist    = dist;
    }
  }

  if min_dist >= _0
  { return None }

  let shift = fixme[best_dir_id].scalar_mul(&min_dist);

  //  FIXME: optimize by translating the last gjk simplex
  match gjk::closest_points::<S, G1, Translated<G2, V>, V, N>
        (g1, &Translated::new(g2, shift))
  {
    None => fail!("The origin was inside of the Simplex during phase 1."),
    Some((p1, p2)) => {
      let corrected_normal = (p1 - p2).normalized();

      let min_dist2 = corrected_normal.dot(&cso.support_point(&corrected_normal));

      assert!(min_dist2 >= _0, "The Minkowski Sampling algorithm must be used" +
                               " only when the origin is inside of the cso.");

      assert!(min_dist >= min_dist2);

      // FIXME: (optim) use a scalar_mul_implace?
      let shift2 = corrected_normal.scalar_mul(&min_dist2);

      //  FIXME: optimize by translating the last gjk simplex
      match gjk::closest_points::<S, G1, Translated<G2, V>, V, N>
            (g1, &Translated::new(g2, copy shift2))
      {
        None =>
          fail!("Internal error: the origin was inside of the Simplex during phase 2."),
        Some((res1, res2)) => Some((res1, res2 - shift2))
      }
    }
  }
}
