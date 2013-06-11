use broad::broad_phase::BroadPhase;

/// Broad phase returning no collision. Used for debugging.
struct EmptyBroadPhase<RB>;

impl<RB> BroadPhase<RB> for EmptyBroadPhase<RB>
{
  fn add(&mut self, _: @mut RB)
  { }

  fn remove(&mut self, _: @mut RB)
  { }

  fn collision_pairs(&mut self, _: &[@mut RB]) -> ~[(@mut RB, @mut RB)]
  { ~[] }
}
