use std::uint::iterate;
use std::num::{Zero, One};
use std::vec::{len, push};
use nalgebra::ndim::dmat::{zero_mat_with_dim};
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::sub_dot::SubDot;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::workarounds::scalar_op::{ScalarMul, ScalarDiv};

pub struct ExplicitJohnsonSimplex<V, T>
{
  points: ~[~V]
}

// FIXME: remove ToStr
impl<V: Copy + SubDot<T> + ScalarMul<T> + ScalarDiv<T> + Zero + Add<V, V> +
ToStr,
     T: Ord + Copy + Clone + Eq + DivisionRing + Ord + ToStr>
    ExplicitJohnsonSimplex<V, T>
{
  pub fn new(initial_point: &V) -> ExplicitJohnsonSimplex<V, T>
  { ExplicitJohnsonSimplex { points: ~[~*initial_point] } }

  pub fn add_point(&mut self, pt: &V)
  { push(&mut self.points, ~*pt) }

  pub fn project_origin(&mut self) -> Option<V>
  {
    let     _0  = Zero::zero::<T>();
    let     _1  = One::one::<T>();
    let     dim = len(self.points);
    let mut mat = zero_mat_with_dim(dim);

    for iterate(0u, dim) |i|
    { mat.set(0u, i, &_1) }

    for iterate(1u, dim) |i|
    {
      for iterate(0u, dim) |j|
      {
        mat.set(
          i,
          j,
          &self.points[i].sub_dot(self.points[0], self.points[j])
        )
      }
    }

    mat.invert();

    let mut res        = Zero::zero::<V>();
    let mut normalizer = Zero::zero::<T>();

    for iterate(0u, dim) |i|
    {
      if (mat.at(i, 0u) > _0)
      {
        let offset = mat.at(i, 0u);
        res        += self.points[i].scalar_mul(&offset);
        normalizer += offset;
        println(~"ADDED: " + self.points[i].to_str());
        println(~"at: " + offset.to_str());
      }
    }

    res.scalar_div_inplace(&normalizer);

    Some(res)
  }
}
