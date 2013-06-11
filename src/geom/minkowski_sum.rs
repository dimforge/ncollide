use geom::implicit::Implicit;

/**
 * Implicit representation of the minkowski sum of two geometries.
 * The only way to obtain the sum points is to use its support mapping
 * function.
 *
 *   `G1`: type of the first object involved on the sum.
 *   `G2`: type of the second object involved on the sum.
 */
#[deriving(Eq, ToStr)]
pub struct MinkowskiSum<G1, G2>
{
  priv g1: @G1,
  priv g2: @G2
}

impl<G1, G2> MinkowskiSum<G1, G2>
{
  /**
   * Builds the Minkowski sum of two geometries. Since the representation is
   * implicit, this is done in constant time.
   */
  pub fn new(g1: @G1, g2: @G2) -> MinkowskiSum<G1, G2>
  { MinkowskiSum { g1: g1, g2: g2 } }
}

impl<V: Add<V, V>, G1: Implicit<V>, G2: Implicit<V>>
Implicit<V> for MinkowskiSum<G1, G2>
{
  fn support_point(&self, dir: &V) -> V
  { self.g1.support_point(dir) + self.g2.support_point(dir) }
}
