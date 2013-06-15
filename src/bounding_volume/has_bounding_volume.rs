pub trait HasBoundingVolume<BV>
{
  fn bounding_volume(&self) -> BV;
}
