use std::managed;
use utils::managed::position_elem_mut_ptr;
use utils::has_proxy::HasProxy;
use broad::broad_phase::BroadPhase;
use bounding_volume::has_bounding_volume::HasBoundingVolume;
use bounding_volume::bounding_volume::LooseBoundingVolume;

pub struct BoundingVolumeProxy<BV>
{ bounding_volume: BV }

pub struct BruteForceBoundigVolumeBroadPhase<RB, BV, N>
{
  priv objects: ~[@mut RB],
  priv panding: ~[@mut RB],
  priv margin:  N
}

impl<RB, BV, N: Copy> BruteForceBoundigVolumeBroadPhase<RB, BV, N>
{
  pub fn new(margin: N) -> BruteForceBoundigVolumeBroadPhase<RB, BV, N>
  {
    BruteForceBoundigVolumeBroadPhase {
      objects: ~[],
      panding: ~[],
      margin:  margin
    }
  }
}

impl<RB: HasProxy<BoundingVolumeProxy<BV>> + HasBoundingVolume<BV>,
     BV: LooseBoundingVolume<N>,
     N:  Copy>
     BroadPhase<RB> for BruteForceBoundigVolumeBroadPhase<RB, BV, N>
{
  fn add(&mut self, rb: @mut RB)
  {
    self.objects.push(rb);
    self.panding.push(rb);
    rb.proxy_mut().bounding_volume = rb.bounding_volume();
    rb.proxy_mut().bounding_volume.loosen(self.margin);
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

    for rbs.each |&b|
    {
      let mut new_bv = b.bounding_volume();

      if !b.proxy().bounding_volume.contains(&new_bv)
      {
        new_bv.loosen(self.margin);
        b.proxy_mut().bounding_volume = new_bv;
        updated.push(b);
      }
    }

    for self.panding.each |&b|
    {
      if position_elem_mut_ptr(updated, b).is_none()
      { updated.push(b) }
    }

    for updated.each |&b1|
    {
      // FIXME: will this do a copy?
      let bv1 = &b1.proxy().bounding_volume;

      for self.objects.each |&b2|
      {
        if !managed::mut_ptr_eq(b1, b2)
        {
          if b2.proxy().bounding_volume.intersects(bv1)
          { res.push((b1, b2)) }
        }
      }
    }

    res
  }
}
