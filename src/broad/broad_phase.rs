pub trait BroadPhase<RB>
{
  // FIXME: type of RB?
  fn add_object(&mut self, &RB);
  fn remove_object(&mut self, &RB);
  fn collision_pairs(&self) -> ~[(RB, RB)];
}
