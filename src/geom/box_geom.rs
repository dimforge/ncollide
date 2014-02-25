//! Support mapping based Box geometry.

use std::num::Signed;
use nalgebra::na::Iterable;
use nalgebra::na;
use math::{N, V};

/// Geometry of a box.
///
/// # Parameters:
///   * N - type of an extent of the box
///   * V - vector of extents. This determines the box dimension
#[deriving(Eq, Show, Clone, Encodable, Decodable)]
pub struct Box {
    priv half_extents: V,
    priv margin:       N
}

impl Box {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Box {
        Box::new_with_margin(half_extents, na::cast(0.04))
    }

    /// Creates a new box from its half-extents and its margin. Half-extents are the box half-width
    /// along each axis. Each half-extent must be greater than the margin.
    #[inline]
    pub fn new_with_margin(half_extents: V, margin: N) -> Box {
        let half_extents_wo_margin = half_extents - margin;
        assert!(half_extents_wo_margin.iter().all(|e| e.is_positive()));

        Box {
            half_extents: half_extents_wo_margin,
            margin:       margin
        }
    }
}

impl Box {
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> V {
        self.half_extents.clone()
    }

    /// The margin surrounding this box.
    ///
    /// Note that unlike most other geometries, a box has an interior margin. Therefore, the
    /// real extents of the box (those that have been passed to the Box constructor) equal the sum
    /// of the margin and its half-extents (as returned by the `half_extents` method).
    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}
