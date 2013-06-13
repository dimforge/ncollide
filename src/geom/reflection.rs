use geom::implicit::Implicit;

/**
 * Implicit represention of the reflection of a geometric object.
 * A reflection is obtained with the central symetry wrt the origin.
 */
#[deriving(Eq)]
pub struct Reflection<G>
{ priv g: @G }

impl<G> Reflection<G>
{
  /// Build the reflection of a geometry. Since the representation is implicit,
  /// the reflection computation is done in constant time.
  #[inline(always)]
  pub fn new(g: @G) -> Reflection<G>
  { Reflection { g: g } }
}

impl<V: Neg<V>, G: Implicit<V>> Implicit<V> for Reflection<G>
{
  #[inline(always)]
  fn support_point(&self, dir: &V) -> V
  { -self.g.support_point(&-dir) }
}
