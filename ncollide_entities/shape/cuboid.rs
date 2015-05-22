//! Support mapping based Cuboid shape.

use na::Iterable;
use na;
use math::Scalar;

/// Shape of a box.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
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
        assert!(half_extents.iter().all(|e| *e >= na::zero()));

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
