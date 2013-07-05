use std::uint;
use std::num::{Zero, One};
use nalgebra::dmat::zero_mat_with_dim;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use nalgebra::traits::dim::Dim;
use narrow::algorithm::simplex::Simplex;

pub struct BruteForceSimplex<V, N>
{
  points: ~[V]
}

impl<V: Clone + VectorSpace<N> + SubDot<N> + Norm<N>,
     N: Ord + Clone + Copy + Eq + DivisionRing + Ord>
    BruteForceSimplex<V, N>
{
  pub fn new(initial_point: V) -> BruteForceSimplex<V, N>
  { BruteForceSimplex { points: ~[initial_point] } }

  pub fn add_point(&mut self, pt: V)
  { self.points.push(pt) }

  fn project_on_subsimplex(points: &[V]) -> Option<V>
  {
    let     _0  = Zero::zero::<N>();
    let     _1  = One::one::<N>();
    let     dim = points.len();
    let mut mat = zero_mat_with_dim(dim);

    for uint::iterate(0u, dim) |i|
    { mat.set(0u, i, &_1) }

    for uint::iterate(1u, dim) |i|
    {
      for uint::iterate(0u, dim) |j|
      {
        mat.set(
          i,
          j,
          &points[i].sub_dot(&points[0], &points[j])
        )
      }
    }

    if !mat.invert()
    { None }
    else
    {
      let mut res        = Zero::zero::<V>();
      let mut normalizer = Zero::zero::<N>();

      for uint::iterate(0u, dim) |i|
      {
        if mat.at(i, 0u) > _0
        {
          let offset = mat.at(i, 0u);
          res        = res + points[i].scalar_mul(&offset);
          normalizer = normalizer + offset;
        }
        else
        { return None }
      }

      res.scalar_div_inplace(&normalizer);

      Some(res)
    }
  }

  fn project_on_subsimplices(points: ~[V]) -> (V, ~[V])
  {
    if points.len() == 1
    { (points[0].clone(), points) }
    else
    {
      let mut bestproj = BruteForceSimplex::project_on_subsimplex(points);
      let mut bestpts  = points.clone();

      for uint::iterate(0u, points.len()) |i|
      {
        let mut subsimplex = ~[];
        for uint::iterate(0u, points.len()) |j|
        {
          if i != j
          { subsimplex.push(points[j].clone()) }
        }

        let (proj, sub_p_pts) = BruteForceSimplex::project_on_subsimplices(subsimplex);

        match bestproj
        {
          Some(ref p) => if p.norm() > proj.norm()
                     { bestpts = sub_p_pts },
          None    => bestpts = sub_p_pts
        }

        bestproj = match bestproj
                   {
                     Some(ref p) => if p.norm() > proj.norm()
                                { Some(proj) }
                                else { bestproj.clone() },
                     None    => Some(proj)
                   }
      }

      (bestproj.unwrap(), bestpts)
    }
  }

  pub fn do_project_origin(&mut self, reduce: bool) -> V
  {
    let (res, reduction) = BruteForceSimplex::project_on_subsimplices(self.points.clone());

    if reduce
    { self.points = reduction }

    res
  }
}

impl<V: Clone + VectorSpace<N> + SubDot<N> + Norm<N> + Eq + Dim,
     N: Ord + Clone + Copy + Eq + DivisionRing + Ord>
Simplex<V, N> for BruteForceSimplex<V, N>
{
  pub fn new(initial_point: V) -> BruteForceSimplex<V, N>
  { BruteForceSimplex::new(initial_point) }

  pub fn dimension(&self) -> uint
  { self.points.len() - 1 }

  pub fn max_sq_len(&self) -> N
  { self.points.iter().transform(|v| v.sqnorm()).max().unwrap() }

  pub fn contains_point(&self, pt: &V) -> bool
  { self.points.iter().any_(|v| pt == v) }

  pub fn add_point(&mut self, pt: V)
  {
    assert!(self.points.len() <= Dim::dim::<V>());
    self.points.push(pt)
  }

  pub fn project_origin_and_reduce(&mut self) -> V
  { self.do_project_origin(true) }

  pub fn project_origin(&mut self) -> V
  { self.do_project_origin(false) }
}
