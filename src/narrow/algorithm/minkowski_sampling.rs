use std::num::Zero;
use std::rand::Rand;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::sample::UniformSphereSample;
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
                      V:  ToStr + Norm<N> + Dot<N> + Dim + Rand + VectorSpace<N> +
                          UniformSphereSample + Clone,
                      N:  ToStr + Sub<N, N> + Ord + Mul<N, N> + Float>
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
  let mut best_dir: Option<&'static V> = None;
  let mut min_dist = Bounded::max_value();
  let mut best_support = Zero::zero(); // FIXME: remove that (for debug)

  do UniformSphereSample::sample::<V>() |sample|
  {
    let support = cso.support_point(sample);
    let dist    = sample.dot(&support);

    if (dist < min_dist)
    {
      best_dir     = Some(sample);
      best_support = support;
      min_dist     = dist;
    }
  }

  if min_dist <= _0
  { return None }

  let shift = best_dir.unwrap().scalar_mul(&min_dist);

  //  FIXME: optimize by translating the last gjk simplex
  match gjk::closest_points::<S, G1, Translated<G2, V>, V, N>
        (g1, &Translated::new(g2, shift.clone()))
  {
    None => None, // fail!("Internal error: the origin was inside of the Simplex during phase 1."),
    Some((p1, p2)) => {
      let corrected_normal = (p2 - p1).normalized();

      let corrected_support = cso.support_point(&corrected_normal);
      let min_dist2 = corrected_normal.dot(&corrected_support);

      // println("min_dist: " + min_dist.to_str());
      // println("min_dist2: " + min_dist2.to_str());
      // println("best_dir: " + best_dir.unwrap().to_str());
      // println("corrected_normal: " + corrected_normal.to_str());
      // println("best_support: " + best_support.to_str());
      // println("corrected_support: " + corrected_support.to_str());
      // assert!(min_dist2 >= _0, "Internal error: corrected normal is invalid.");
      if min_dist2 < _0
      { return None }

      let shift2 = corrected_normal.scalar_mul(&min_dist2);

      Some((p2 - shift + shift2 , p2 - shift))
      /* This might not really be useful after all
      else
      {
        // FIXME: (optim) use a scalar_mul_implace?

        //  FIXME: optimize by translating the last gjk simplex
        match gjk::closest_points::<S, G1, Translated<G2, V>, V, N>
              (g1, &Translated::new(g2, shift2.clone()))
        {
          None =>
            fail!("Internal error: the origin was inside of the Simplex during phase 2."),
          Some((res1, res2)) => Some((res1, res2 - shift2))
        }
      }
      */
    }
  }
}
