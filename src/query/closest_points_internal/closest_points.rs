use crate::math::Point;
use na::Real;
use std::mem;

/// Closest points information.
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum ClosestPoints<N: Real> {
    /// The two objects are intersecting.
    Intersecting,
    /// The two objects are non-intersecting but closer than a given user-defined distance.
    WithinMargin(Point<N>, Point<N>),
    /// The two objects are non-intersecting and further than a given user-defined distance.
    Disjoint,
}

impl<N: Real> ClosestPoints<N> {
    /// Swaps the two points.
    pub fn flip(&mut self) {
        if let ClosestPoints::WithinMargin(ref mut p1, ref mut p2) = *self {
            mem::swap(p1, p2)
        }
    }
}
