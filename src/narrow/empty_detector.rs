use narrow::collision_detector::CollisionDetector;

/// Collision detector working with any geometry. It will never report any
/// collision. Used for debug purpoise.
/// Use it if you want to benchmark the broad phase.
pub struct EmptyDetector;

impl<C, G1, G2> CollisionDetector<C, G1, G2> for EmptyDetector
{
  #[inline]
  fn new(_: &G1, _: &G2) -> EmptyDetector
  { EmptyDetector }

  #[inline]
  fn update(&mut self, _: &G1, _: &G2)
  { }

  #[inline]
  fn num_coll(&self) -> uint
  { 0u }

  #[inline]
  fn colls(&mut self, _: &mut ~[@mut C])
  { }
}
