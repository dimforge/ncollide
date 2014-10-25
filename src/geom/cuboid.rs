//! Support mapping based Cuboid geometry.

use std::num::Signed;
use na::Iterable;
use math::Scalar;

/// Geometry of a box.
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Cuboid<V> {
    half_extents: V
}

impl<N, V> Cuboid<V>
    where N: Scalar,
          V: Iterable<N> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Cuboid<V> {
        assert!(half_extents.iter().all(|e| e.is_positive()));

        Cuboid {
            half_extents: half_extents
        }
    }
}

impl<V> Cuboid<V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis.
    #[inline]
    pub fn half_extents(&self) -> &V {
        &self.half_extents
    }
}
