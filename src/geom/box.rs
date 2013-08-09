//!
//! Support mapping based Box geometry.
//!

use std::num::{Zero, Signed};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::indexable::Indexable;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::iterable::Iterable;
use nalgebra::traits::scalar_op::{ScalarMul, ScalarDiv};
use nalgebra::traits::dot::Dot;
use nalgebra::traits::basis::Basis;
use bounding_volume::aabb::{HasAABB, AABB};
use bounding_volume::aabb;
use geom::implicit::Implicit;

/// Geometry of a box.
///
/// # Parameters:
///   * N - type of an extent of the box
///   * V - vector of extents. This determines the box dimension
#[deriving(Eq, ToStr, Clone)]
pub struct Box<N, V> {
    priv half_extents: V
}

impl<N: Signed, V: Iterable<N>> Box<N, V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be positive but not zero.
    #[inline]
    pub fn new(half_extents: V) -> Box<N, V> {
        assert!(half_extents.iter().all(|e| e.is_positive()));

        Box {
            half_extents: half_extents
        }
    }
}

impl<N, V: Clone> Box<N, V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> V {
        self.half_extents.clone()
    }
}

impl<N: Signed,
     V: Dim + Indexable<uint, N> + Zero,
     M: Rotate<V> + Transform<V>>
Implicit<V, M> for Box<N, V> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> V {
        let local_dir = m.inv_rotate(dir);

        let mut vres = Zero::zero::<V>();

        for i in range(0u, Dim::dim::<V>()) {
            if local_dir.at(i).is_negative() {
                vres.set(i, -self.half_extents.at(i));
            }
            else {
                vres.set(i, self.half_extents.at(i));
            }
        }

        m.transform_vec(&vres)
    }
}

impl<N: Signed,
     V: Dim + Indexable<uint, N> + Zero + Dot<N> + ScalarMul<N> + ScalarDiv<N> +
        Basis + Neg<V> + Add<V, V> + Ord,
     M: Rotate<V> + Transform<V>>
HasAABB<N, V, M> for Box<N, V> {
    fn aabb(&self, m: &M) -> AABB<N, V> {
        aabb::implicit_shape_aabb(m, self)
    }
}
