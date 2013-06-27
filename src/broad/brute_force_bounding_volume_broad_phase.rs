use std::managed;
use utils::default::Default;
use utils::managed::position_elem_mut_ptr;
use broad::broad_phase::BroadPhase;
use bounding_volume::has_bounding_volume::HasBoundingVolume;
use bounding_volume::bounding_volume::LooseBoundingVolume;

pub trait HasBoundingVolumeProxy<BV>
{
  fn proxy<'r>(&'r self)         -> &'r BoundingVolumeProxy<BV>;
  fn proxy_mut<'r>(&'r mut self) -> &'r mut BoundingVolumeProxy<BV>;
}

#[deriving(ToStr)]
pub struct BoundingVolumeProxy<BV>
{ bounding_volume: BV }

impl<BV: Default> Default for BoundingVolumeProxy<BV>
{
  fn default() -> BoundingVolumeProxy<BV>
  { BoundingVolumeProxy { bounding_volume: Default::default() } }
}

pub struct BruteForceBoundingVolumeBroadPhase<RB, BV, N>
{
  priv objects: ~[@mut RB],
  priv panding: ~[@mut RB],
  priv margin:  N
}

impl<RB, BV, N: Copy> BruteForceBoundingVolumeBroadPhase<RB, BV, N>
{
  pub fn new(margin: N) -> BruteForceBoundingVolumeBroadPhase<RB, BV, N>
  {
    BruteForceBoundingVolumeBroadPhase {
      objects: ~[],
      panding: ~[],
      margin:  margin
    }
  }
}

impl<RB: HasBoundingVolumeProxy<BV> + HasBoundingVolume<BV>,
     BV: LooseBoundingVolume<N>,
     N:  Copy>
     BroadPhase<RB> for BruteForceBoundingVolumeBroadPhase<RB, BV, N>
{
  fn add(&mut self, rb: @mut RB)
  {
    self.objects.push(rb);
    self.panding.push(rb);
    rb.proxy_mut().bounding_volume = rb.bounding_volume();
    rb.proxy_mut().bounding_volume.loosen(copy self.margin);
  }

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

  fn collision_pairs(&mut self, rbs: &[@mut RB]) -> ~[(@mut RB, @mut RB)]
  {
    let mut res     = ~[];
    let mut updated = ~[];

    for rbs.iter().advance |&b|
    {
      let mut new_bv = b.bounding_volume();

      if !b.proxy().bounding_volume.contains(&new_bv)
      {
        new_bv.loosen(copy self.margin);
        b.proxy_mut().bounding_volume = new_bv;
        updated.push(b);
      }
    }

    for self.panding.iter().advance |&b|
    {
      if position_elem_mut_ptr(updated, b).is_none()
      { updated.push(b) }
    }

    for updated.iter().advance |&b1|
    {
      let bv1 = &b1.proxy().bounding_volume;

      for self.objects.iter().advance |&b2|
      {
        if !managed::mut_ptr_eq(b1, b2)
        {
          if b2.proxy().bounding_volume.intersects(bv1)
          { res.push((b1, b2)) }
        }
      }
    }

    self.panding.clear();

    res
  }
}
