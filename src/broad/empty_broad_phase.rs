use broad::broad_phase::BroadPhase;

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
