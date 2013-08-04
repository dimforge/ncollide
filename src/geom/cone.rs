//!
//! Support mapping based Cone geometry.
//!

use std::num::Zero;
use nalgebra::traits::transformation::Transformable;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::scalar_op::ScalarMul;
use geom::implicit::Implicit;
use geom::transformed::Transformed;

/// Implicit description of a cylinder geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Cone<N>
{
    priv half_height: N,
    priv radius: N
}

impl<N: Signed> Cone<N>
{
    /// Creates a new cone.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the cone along the `x` axis.
    ///     * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N>
    {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius: radius
        }
    }
}

impl<N: Clone> Cone<N>
{
    /// The cone half length along the `x` axis.
    pub fn half_height(&self) -> N
    { self.half_height.clone() }

    /// The radius of the cone along all but the `x` axis.
    pub fn radius(&self) -> N
    { self.radius.clone() }
}

impl<N: Clone + Signed,
     V: Clone + Zero + Norm<N> + ScalarMul<N> + Indexable<uint, N>> Implicit<V> for Cone<N>
{
    fn support_point(&self, dir: &V) -> V
    {
        if dir.at(0).is_negative() // points toward the base
        {
            let mut vres = dir.clone();

            vres.set(0, Zero::zero());

            if vres.normalize().is_zero()
            { vres = Zero::zero() }
            else
            { vres.scalar_mul_inplace(&self.radius) }

            vres.set(0, -self.half_height);

            vres
        }
        else // points toward the pointy thing
        {
            let mut vres = Zero::zero::<V>();

            vres.set(0, self.half_height.clone());

            vres
        }
    }
}

impl<N: Clone, M: Clone + Mul<M, M> + Inv>
Transformable<M, Transformed<Cone<N>, M, N>> for Cone<N>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Transformed<Cone<N>, M, N>
  { Transformed::new(transform.clone(), self.clone()) }
}
