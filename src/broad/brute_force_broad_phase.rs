use std::managed;
use utils::managed::position_elem_mut_ptr;
use broad::broad_phase::BroadPhase;

/**
 * The slowest possible broad phase. It always returns false positive since
 * it assumes that all object is in collision with all objects.
 * Do not use this but for benchmarking the narrow phase.
 */
struct BruteForceBroadPhase<RB>
{
  priv objects: ~[@mut RB],
  priv panding: ~[@mut RB]
}

impl<RB> BruteForceBroadPhase<RB>
{
  /// Builds a new brute force broad phase.
  pub fn new() -> BruteForceBroadPhase<RB>
  {
    BruteForceBroadPhase {
      objects: ~[],
      panding: ~[]
    }
  }
}

impl<RB> BroadPhase<RB> for BruteForceBroadPhase<RB>
{
  fn add(&mut self, b: @mut RB)
  { self.panding.push(b); }

  fn remove(&mut self, b: @mut RB)
  {
    match position_elem_mut_ptr(self.objects, b)
    {
      None => {
        match position_elem_mut_ptr(self.panding, b)
        {
          None    => fail!("Tried to remove an unexisting element."),
          Some(i) => self.panding.remove(i)
        }
      },

      Some(i) => self.objects.remove(i)
    };
  }

  fn collision_pairs(&mut self, _: &[@mut RB]) -> ~[(@mut RB, @mut RB)]
  {
    let mut res: ~[(@mut RB, @mut RB)] = ~[];

    for self.panding.each |&o|
    {
      for self.objects.each |&o2|
      { res.push((o, o2)) }

      for self.panding.each |&o2|
      {
        if (!managed::mut_ptr_eq(o, o2))
        { res.push((o, o2)) }
      }
    }

    for self.panding.each |&o|
    { self.objects.push(o) }

    self.panding.clear();

    res
  }
}
