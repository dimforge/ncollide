use std::mem;

/// Closest points information.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum ClosestPoints<P> {
    /// The two objects are intersecting.
    Intersecting,
    /// The two objects are non-intersecting but closer than a given user-defined distance.
    WithinMargin(P, P),
    /// The two objects are non-intersecting and further than a given user-defined distance.
    Disjoint,
}

impl<P> ClosestPoints<P> {
    /// Swaps the two points.
    pub fn flip(&mut self) {
        if let ClosestPoints::WithinMargin(ref mut p1, ref mut p2) = *self {
            mem::swap(p1, p2)
        }
    }
}
