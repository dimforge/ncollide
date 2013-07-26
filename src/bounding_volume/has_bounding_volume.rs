use bounding_volume::bounding_volume::BoundingVolume;

// FIXME: move this to bounding_volume.rs
pub trait HasBoundingVolume<BV: BoundingVolume>
{
  fn bounding_volume(&self) -> BV;
}
