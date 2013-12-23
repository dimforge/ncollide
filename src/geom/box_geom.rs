//!
//! Support mapping based Box geometry.
//!

use std::num::Signed;
use nalgebra::na::{Cast, Iterable, VecExt};
use nalgebra::na;

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

impl<N: Signed + Cast<f32>, V: VecExt<N>> Box<N, V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Box<N, V> {
        Box::new_with_margin(half_extents, na::cast(0.04))
    }

    /// Creates a new box from its half-extents and its margin. Half-extents are the box half-width
    /// along each axis. Each half-extent must be greater than the margin.
    #[inline]
    pub fn new_with_margin(half_extents: V, margin: N) -> Box<N, V> {
        let half_extents_wo_margin = half_extents.sub_s(&margin);
        assert!(half_extents_wo_margin.iter().all(|e| e.is_positive()));

        Box {
            half_extents: half_extents_wo_margin,
            margin:       margin
        }
    }
}

impl<N: Clone, V: Clone> Box<N, V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis. 
    #[inline]
    pub fn half_extents(&self) -> V {
        self.half_extents.clone()
    }

    #[inline]
    pub fn margin(&self) -> N {
        self.margin.clone()
    }
}
