pub trait BroadPhase<RB>
{
  // FIXME: is '@mut RB' the best? Could using only 'RB' be more flexible
  fn add(&mut self, @mut RB);
  fn remove(&mut self, @mut RB);
  fn collision_pairs(&mut self, &[@mut RB]) -> ~[(@mut RB, @mut RB)];
}
