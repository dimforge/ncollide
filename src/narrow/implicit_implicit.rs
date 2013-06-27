use std::num::Zero;
use std::rand::Rand;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use geom::implicit::Implicit;
use geom::minkowski_sum::AnnotatedPoint;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::gjk;
use narrow::algorithm::minkowski_sampling;
use narrow::collision_detector::CollisionDetector;
use contact::contact::Contact;
use contact::contact;

pub struct ImplicitImplicitCollisionDetector<S, C, G1, G2, V, N>
{
  priv contact: Option<@mut C>
}

impl<S:  Simplex<AnnotatedPoint<V>, N>,
     G1: Implicit<V>,
     G2: Implicit<V>,
     V:  Norm<N> + VectorSpace<N> + Dot<N> + Dim + Rand + Copy,
     N:  Sub<N, N> + Ord + Mul<N, N> + Float + Copy,
     C:  Contact<V, N>>
    CollisionDetector<C, G1, G2> for
    ImplicitImplicitCollisionDetector<S, C, G1, G2, V, N>
{
  #[inline(always)]
  fn new(_: &G1, _: &G2) -> ImplicitImplicitCollisionDetector<S, C, G1, G2, V, N>
  { ImplicitImplicitCollisionDetector { contact: None } }

  fn update(&mut self, a: &G1, b: &G2)
  {
    match self.contact
    {
      None    => self.contact = collide_implicit_implicit::<S, G1, G2, V, N, C>(a, b).map(|&c| @mut c),
      Some(c) => if !update_collide_implicit_implicit::<S, G1, G2, V, N, C>(a, b, c)
                 { self.contact = None }
    }
  }

  #[inline(always)]
  fn num_coll(&self) -> uint
  {
    match self.contact
    {
      None    => 0,
      Some(_) => 1
    }
  }

  #[inline(always)]
  fn colls(&mut self, out_colls: &mut ~[@mut C])
  {
    match self.contact
    {
      Some(c) => out_colls.push(c),
      None    => ()
    }
  }
}

pub fn update_collide_implicit_implicit
       <S:  Simplex<AnnotatedPoint<V>, N>,
        G1: Implicit<V>,
        G2: Implicit<V>,
        V:  Norm<N> + VectorSpace<N> + Dot<N> + Dim + Rand + Copy,
        N:  Sub<N, N> + Ord + Mul<N, N> + Float + NumCast,
        C:  Contact<V, N>>
       (g1: &G1, g2: &G2, out: &mut C) -> bool
{
  // FIXME: the margin should not be hard-coded…
  let _margin: N = NumCast::from(0.8f64);

  match gjk::closest_points::<S, G1, G2, V, N>(g1, g2)
  {
    Some((p1, p2)) => if (p2 - p1).sqnorm() >= _margin * _margin
                      { false }
                      else
                      {
                        let _0_5: N = NumCast::from(0.5f64);
                        let _2  : N = NumCast::from(2.0f64);
                        let normal = p2 - p1;
                        let depth  = (p2 - p1).normalize();
                        contact::set(out,
                                     p1 + normal.scalar_mul(&_0_5),
                                     p2 + normal.scalar_mul(&-_0_5),
                                     normal,
                                     depth - _2 * _margin);

                        true
                      },
    None => {
      // The point is inside of the CSO: use the fallback algorithm
      match minkowski_sampling::closest_points::<S, G1, G2, V, N>(g1, g2, Zero::zero())
      {
        Some((p1, p2)) => {
          let normal = p2 - p1;
          let depth  = (p2 - p1).normalize();

          contact::set(out, p1, p2, normal, depth);

          true
        }
        None => fail!("Both GJK and fallback algorithm failed.")
      }
    }
  }
}

pub fn collide_implicit_implicit
       <S:  Simplex<AnnotatedPoint<V>, N>,
        G1: Implicit<V>,
        G2: Implicit<V>,
        V:  Norm<N> + VectorSpace<N> + Dot<N> + Dim + Rand + Copy,
        N:  Sub<N, N> + Ord + Mul<N, N> + Float + Copy,
        C:  Contact<V, N>>
      (g1: &G1, g2: &G2) -> Option<C>
{
  let mut res = contact::zero::<V, N, C>();

  if update_collide_implicit_implicit::<S, G1, G2, V, N, C>(g1, g2, &mut res)
  { Some(res) }
  else
  { None }
}
