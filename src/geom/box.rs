//!
//! Support mapping based Box geometry.
//!

use std::num::{Zero, Signed};
use nalgebra::vec::{Dim, Indexable, Iterable, VecExt, AlgebraicVecExt, ScalarSub};
use nalgebra::mat::{Transform, Rotate};
use bounding_volume::{HasAABB, AABB};
use bounding_volume;
use geom::{HasMargin, Implicit};

/// Geometry of a box.
///
/// # Parameters:
///   * N - type of an extent of the box
///   * V - vector of extents. This determines the box dimension
#[deriving(Eq, ToStr, Clone, Encodable, Decodable)]
pub struct Box<N, V> {
    priv half_extents: V,
    priv margin:       N
}

impl<N: Signed + NumCast, V: VecExt<N>> Box<N, V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be positive but not zero.
    #[inline]
    pub fn new(half_extents: V) -> Box<N, V> {
        Box::new_with_margin(half_extents, NumCast::from(0.04f64))
    }

    /// Creates a new box from its half-extents and its margin. Half-extents are the box half-width
    /// along each axis. Each half-extent must be positive but not zero.
    #[inline]
    pub fn new_with_margin(half_extents: V, margin: N) -> Box<N, V> {
        let half_extents_wo_margin = half_extents.scalar_sub(&margin);
        assert!(half_extents_wo_margin.iter().all(|e| e.is_positive()));

        Box {
            half_extents: half_extents_wo_margin,
            margin:       margin
        }
    }
}

impl<N, V: VecExt<N> + Clone> Box<N, V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> V {
        self.half_extents.scalar_add(&self.margin)
    }
}

impl<N: Clone, V> HasMargin<N> for Box<N, V> {
    #[inline]
    fn margin(&self) -> N {
        self.margin.clone()
    }
}


impl<N: Algebraic + Signed + Clone,
     V: AlgebraicVecExt<N>,
     M: Rotate<V> + Transform<V>>
Implicit<N, V, M> for Box<N, V> {
    #[inline]
    fn support_point_without_margin(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres: V = Zero::zero();

        for i in range(0u, Dim::dim(None::<V>)) {
            if local_dir.at(i).is_negative() {
                vres.set(i, -self.half_extents.at(i));
            }
            else {
                vres.set(i, self.half_extents.at(i));
            }
        }

        m.transform(&vres)
    }
}

impl<N: Algebraic + Signed + Clone,
     V: AlgebraicVecExt<N>,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Box<N, V> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        bounding_volume::implicit_shape_aabb(m, self)
    }
}
