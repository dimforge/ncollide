#[test]
use std::rand;
#[test]
use std::uint;
#[test]
use nalgebra::traits::dim::{Dim, d4, d5, d6};
#[test]
use nalgebra::traits::scalar_op::{ScalarSub, ScalarMul};
#[test]
use nalgebra::dim3::vec3::Vec3;
#[test]
use nalgebra::dim2::vec2::Vec2;
#[test]
use nalgebra::dim1::vec1::Vec1;
#[test]
use nalgebra::ndim::nvec::NVec;
#[test]
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
#[test]
use narrow::algorithm::brute_force_simplex::BruteForceSimplex;

macro_rules! test_johnson_simplex_impl(
  ($t: ty, $n: ty) => (
    for uint::iterate(0, Dim::dim::<$t>() + 1) |d|
    {
      for uint::iterate(1, 200 / (d + 1)) |i|
      {
        // note that this fails with lower precision
        let mut v1: $t = rand::random();
        v1.scalar_sub_inplace(&(0.5 as $n));
        v1.scalar_mul_inplace(&(i   as $n));

        let mut splx1 = JohnsonSimplex::new(&v1);
        let mut splx2 = BruteForceSimplex::new(&v1);

        for d.times
        {
          let mut v: $t = rand::random();
          v.scalar_sub_inplace(&(0.5 as $n));
          v.scalar_mul_inplace(&(i   as $n));

          splx1.add_point(&v);
          splx2.add_point(&v);
        }

        let proj2 = splx2.project_origin();
        let proj1 = splx1.project_origin();

        assert!(proj1.approx_eq(&proj2));
      }
    }
  )
)

#[test]
fn test_johnson_simplex_1d()
{ test_johnson_simplex_impl!(Vec1<f64>, f64); }

#[test]
fn test_johnson_simplex_2d()
{ test_johnson_simplex_impl!(Vec2<f64>, f64); }

#[test]
fn test_johnson_simplex_3d()
{ test_johnson_simplex_impl!(Vec3<f64>, f64); }

#[test]
fn test_johnson_simplex_4d()
{ test_johnson_simplex_impl!(NVec<d4, f64>, f64); }

#[test]
fn test_johnson_simplex_5d()
{ test_johnson_simplex_impl!(NVec<d5, f64>, f64); }

#[test]
fn test_johnson_simplex_6d()
{ test_johnson_simplex_impl!(NVec<d6, f64>, f64); }
