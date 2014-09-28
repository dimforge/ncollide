//! Support mapping based Cuboid geometry.

use std::num::Signed;
use na::Iterable;
use na;
use math::{Scalar, Vect};

/// Geometry of a box.
///
/// # Parameters:
///   * Scalar - type of an extent of the box
///   * Vect - vector of extents. This determines the box dimension
#[deriving(PartialEq, Show, Clone, Encodable, Decodable)]
pub struct Cuboid {
    half_extents: Vect,
}

impl Cuboid {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: Vect) -> Cuboid {
        assert!(half_extents.iter().all(|e| e.is_positive()));

        Cuboid {
            half_extents: half_extents
        }
    }
}

impl Cuboid {
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> Vect {
        self.half_extents.clone()
    }
}
