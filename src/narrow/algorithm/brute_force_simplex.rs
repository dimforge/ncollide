use std::uint;
use std::num::{Zero, One};
use nalgebra::ndim::dmat::{zero_mat_with_dim};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};

pub struct BruteForceSimplex<V, N>
{
  points: ~[~V]
}

// FIXME: remove ToStr
impl<V: Copy         +
        SubDot<N>    +
        ScalarMul<N> +
        ScalarDiv<N> +
        Zero         +
        Add<V, V>    +
        Norm<N>      +
        ToStr,
     N: Ord          +
        Copy         +
        Clone        +
        Eq           +
        DivisionRing +
        Ord          +
        ToStr>
    BruteForceSimplex<V, N>
{
  pub fn new(initial_point: &V) -> BruteForceSimplex<V, N>
  { BruteForceSimplex { points: ~[~copy *initial_point] } }

  pub fn add_point(&mut self, pt: &V)
  { self.points.push(~copy *pt) }

  fn project_on_subsimplex(points: &[~V]) -> Option<V>
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
          &points[i].sub_dot(points[0], points[j])
        )
      }
    }

    mat.invert();

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

  fn project_on_subsimplices(points: &[~V]) -> V
  {
    if points.len() == 1
    { copy *points[0] }
    else
    {
      let mut bestproj = BruteForceSimplex::project_on_subsimplex(points);

      for uint::iterate(0u, points.len()) |i|
      {
        let mut subsimplex = ~[];
        for uint::iterate(0u, points.len()) |j|
        {
          if i != j
          { subsimplex.push(~copy *points[j]) }
        }

        let proj = BruteForceSimplex::project_on_subsimplices(subsimplex);

        bestproj =
        match bestproj
        {
          Some(ref p) => if p.norm() > proj.norm() { Some(copy proj) }
                         else { Some(copy *p) },
          None        => Some(proj)
        }
      }

      bestproj.unwrap()
    }
  }

  pub fn project_origin(&mut self) -> V
  {
    BruteForceSimplex::project_on_subsimplices(self.points)
  }

}
