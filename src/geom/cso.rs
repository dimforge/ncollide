use geom::minkowski_sum::{MinkowskiSum, AnnotatedMinkowskiSum};
use geom::reflection::Reflection;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<'self, G1, G2> = MinkowskiSum<'self, G1, Reflection<'self, G2>>;
pub type AnnotatedCSO<'self, G1, G2> = AnnotatedMinkowskiSum<'self, G1, Reflection<'self, G2>>;
