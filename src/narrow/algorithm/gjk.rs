use std::num::{Zero, NumCast};
use std::rand::{Rand, random};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::division_ring::DivisionRing;
use geom::implicit::Implicit;
use geom::reflection::Reflection;
use geom::minkowski_sum::{AnnotatedPoint, AnnotatedMinkowskiSum};
use geom::cso::AnnotatedCSO;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use narrow::algorithm::simplex::Simplex;

pub fn closest_points_johnson<G1: Implicit<V>,
                              G2: Implicit<V>,
                              V:  Norm<N> + SubDot<N> + VectorSpace<N> +
                                  Dim + Rand + Copy + Eq,
                              N:  Ord + DivisionRing + Bounded + Float + Eq>
                              (g1: &G1, g2: &G2) -> Option<(V, V)>
{ closest_points::<JohnsonSimplex<AnnotatedPoint<V>, N>, G1, G2, V, N>(g1, g2) }

pub fn closest_points_with_initial_direction_johnson
       <G1: Implicit<V>,
        G2: Implicit<V>,
        V:  Norm<N> + SubDot<N> + VectorSpace<N> +
            Dim + Rand + Copy + Eq,
        N:  Ord + DivisionRing + Bounded + Float + Eq>
       (g1: &G1, g2: &G2, dir: V) -> Option<(V, V)>
{
  closest_points_with_initial_direction
  ::<JohnsonSimplex<AnnotatedPoint<V>, N>, G1, G2, V, N>(g1, g2, dir)
}

pub fn closest_points<S:  Simplex<AnnotatedPoint<V>, N>,
                      G1: Implicit<V>,
                      G2: Implicit<V>,
                      V:  Norm<N> + Neg<V> + Add<V, V> + Dot<N> + Dim + Rand + Zero + Copy,
                      N:  Sub<N, N> + Ord + Mul<N, N> + Float>
                     (g1: &G1, g2: &G2) -> Option<(V, V)>
{ closest_points_with_initial_direction::<S, G1, G2, V, N>(g1, g2, random()) }

pub fn closest_points_with_initial_direction
       <S:  Simplex<AnnotatedPoint<V>, N>,
        G1: Implicit<V>,
        G2: Implicit<V>,
        V:  Norm<N> + Neg<V> + Add<V, V> + Dot<N> + Dim + Zero + Copy,
        N:  Sub<N, N> + Ord + Mul<N, N> + Float>
       (g1: &G1, g2: &G2, dir: V) -> Option<(V, V)>
{
  let rg2 = &Reflection::new(g2);
  let cso = AnnotatedMinkowskiSum::new(g1, rg2);

  match project_origin_with_initial_direction
        ::<S, AnnotatedCSO<G1, G2>, AnnotatedPoint<V>, N>(&cso, AnnotatedPoint::new_invalid(dir))
  {
    Some(ref pt) => Some((copy *pt.orig1(), -pt.orig2())),
    None         => None
  }
}

pub fn project_origin<S: Simplex<V, N>,
                      G: Implicit<V>,
                      V: Norm<N> + Neg<V> + Dot<N> + Dim + Rand,
                      N: Sub<N, N> + Ord + Mul<N, N> + Float>
                      (g: &G) -> Option<V>
{ project_origin_with_initial_direction::<S, G, V, N>(g, random()) }

pub fn project_origin_with_initial_direction<S: Simplex<V, N>,
                                             G: Implicit<V>,
                                             V: Norm<N> + Neg<V> + Dot<N> + Dim,
                                             N: Sub<N, N> + Ord + Mul<N, N> + Float>
                                            (geom: &G, dir: V) -> Option<V>
{
  let support_point = geom.support_point(&dir);

  project_origin_with_simplex(geom, &mut Simplex::new::<V, N, S>(support_point))
}

// The main algorithm
pub fn project_origin_with_simplex<S: Simplex<V, N>,
                                   G: Implicit<V>,
                                   V: Norm<N> + Neg<V> + Dot<N> + Dim,
                                   N: Sub<N, N> + Ord + Mul<N, N> + Float + NumCast>
                                  (geom: &G, simplex: &mut S) -> Option<V>
{
  let mut proj       = simplex.project_origin_and_reduce();
  let mut sq_len_dir = proj.sqnorm();

  let _eps_tol = Float::epsilon::<N>() * NumCast::from(100.0f64);
  let _eps_rel = Float::epsilon::<N>(); // FIXME: .sqrt();
  let _dim     = Dim::dim::<V>();

  loop
  {
    let support_point = geom.support_point(&-proj);

    if (sq_len_dir - proj.dot(&support_point) <= _eps_rel * sq_len_dir)
    { return Some(proj) } // the distance found has a good enough precision 

    simplex.add_point(support_point);

    proj = simplex.project_origin_and_reduce();

    let old_sq_len_dir = sq_len_dir;

    sq_len_dir = proj.sqnorm();

    if (simplex.dimension() == _dim || sq_len_dir <= _eps_tol * simplex.max_sq_len())
    { return None } // point inside of the cso

    if (sq_len_dir >= old_sq_len_dir) // upper bounds inconsistencies
    {
      // FIXME: we should return the last projection instead
      return Some(proj)
    }
  }
}
