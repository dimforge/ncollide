use geom::minkowski_sum::MinkowskiSum;
use geom::reflection::Reflection;

pub type CSO<G1, G2> = MinkowskiSum<G1, Reflection<G2>>;
