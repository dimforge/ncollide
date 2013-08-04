//!
//! Support mapping based Capsule geometry.
//!

use std::num::Zero;
use nalgebra::traits::transformation::Transformable;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::inv::Inv;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::scalar_op::ScalarMul;
use geom::implicit::Implicit;
use geom::transformed::Transformed;

/// Implicit description of a capsule geometry with its principal axis aligned with the `x` axis.
#[deriving(Eq, ToStr, Clone)]
pub struct Capsule<N>
{
    priv half_height: N,
    priv radius:      N
}

impl<N: Signed> Capsule<N>
{
    /// Creates a new capsule.
    ///
    /// # Arguments:
    ///     * `half_height` - the half length of the capsule along the `x` axis.
    ///     * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N>
    {
        assert!(half_height.is_positive() && radius.is_positive());

        Capsule {
            half_height: half_height,
            radius:      radius
        }
    }
}

impl<N: Clone> Capsule<N>
{
    /// The capsule half length along the `x` axis.
    pub fn half_height(&self) -> N
    { self.half_height.clone() }

    /// The radius of the capsule's rounded part.
    pub fn radius(&self) -> N
    { self.radius.clone() }
}

impl<N: Clone + Signed,
     V: Clone + Zero + Norm<N> + ScalarMul<N> + Indexable<uint, N>> Implicit<V> for Capsule<N>
{
    fn support_point(&self, dir: &V) -> V
    {
        let mut vres = dir.clone();

        let negative = dir.at(0).is_negative();

        vres.scalar_mul_inplace(&self.radius);

        let v0 = vres.at(0);

        if negative
        { vres.set(0, v0 - self.half_height) }
        else
        { vres.set(0, v0 + self.half_height.clone()) }

        vres
    }
}

impl<N: Clone, M: Clone + Mul<M, M> + Inv>
Transformable<M, Transformed<Capsule<N>, M, N>> for Capsule<N>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Transformed<Capsule<N>, M, N>
  { Transformed::new(transform.clone(), self.clone()) }
}
