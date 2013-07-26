use nalgebra::traits::translation::{Translation, Translatable};
use nalgebra::traits::transformation::{Transformation, Transformable};
use geom::implicit::Implicit;

#[deriving(Eq, ToStr, Clone)]
pub struct Translated<'self, G, V>
{
  priv t: V,
  priv g: &'self G
}

impl<'self, G, V> Translated<'self, G, V>
{
  #[inline]
  pub fn new(g: &'self G, t: V) -> Translated<'self, G, V>
  { Translated { g: g, t: t } }
}

impl<'self, V: Add<V, V>, G: Implicit<V>> Implicit<V> for Translated<'self, G, V>
{
  #[inline]
  fn support_point(&self, dir: &V) -> V
  { self.g.support_point(dir) + self.t }
}

// this might do some very interesting structural optimizations
impl<'self,
     G: Transformable<M, Res>,
     V,
     M: Translatable<V, M> + Translation<V>,
     Res: Transformation<M>>
Transformable<M, Res> for Translated<'self, G, V>
{
  #[inline]
  fn transformed(&self, transform: &M) -> Res
  { self.g.transformed(&transform.translated(&self.t)) }
}
