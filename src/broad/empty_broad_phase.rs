use broad::broad_phase::BroadPhase;

struct EmptyBroadPhase<RB>;

impl<RB> BroadPhase<RB> for EmptyBroadPhase<RB>
{
  fn add_object(&mut self, _: &RB)
  { }

  fn remove_object(&mut self, _: &RB)
  { }

  fn collision_pairs(&self) -> ~[(RB, RB)]
  { ~[] }
}
