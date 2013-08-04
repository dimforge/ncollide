//!
//! Support mapping based Box geometry.
//!

use std::num::{Zero, Signed};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::transformation::Transformable;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::inv::Inv;
use geom::implicit::Implicit;
use geom::transformed::Transformed;

/// Geometry of a box.
///
/// # Parameters:
///   * N - type of an extent of the box
///   * V - vector of extents. This determines the box dimension
#[deriving(Eq, ToStr, Clone)]
pub struct Box<N, V>
{ priv half_extents: V }

impl<N: Signed, V: Iterable<N>> Box<N, V>
{
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be positive but not zero.
    #[inline]
    pub fn new(half_extents: V) -> Box<N, V>
    {
        assert!(half_extents.iter().all(|e| e.is_positive()));

        Box { half_extents: half_extents }
    }
}

impl<N, V: Clone> Box<N, V>
{
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> V
    { self.half_extents.clone() }
}

impl<N: Signed, V: Dim + Indexable<uint, N> + Zero> Implicit<V> for Box<N, V>
{
    #[inline]
    fn support_point(&self, dir: &V) -> V
    {
        let mut vres = Zero::zero::<V>();

        for i in range(0u, Dim::dim::<V>())
        {
            if dir.at(i).is_negative()
            { vres.set(i, -self.half_extents.at(i)); }
            else
            { vres.set(i, self.half_extents.at(i)); }
        }

        vres
    }
}

impl<N: Clone, V: Clone, M: Clone + Mul<M, M> + Inv>
Transformable<M, Transformed<Box<N, V>, M, N>> for Box<N, V>
{
    #[inline]
    fn transformed(&self, transform: &M) -> Transformed<Box<N, V>, M, N>
    { Transformed::new(transform.clone(), self.clone()) }
}
