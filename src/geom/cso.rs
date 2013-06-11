use geom::minkowski_sum::MinkowskiSum;
use geom::reflection::Reflection;

/// Type of an implicit representation of the Configuration Space Obstacle
/// formed by two geometric objects.
pub type CSO<G1, G2> = MinkowskiSum<G1, Reflection<G2>>;
