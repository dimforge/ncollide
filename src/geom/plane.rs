#[deriving(Eq)]
pub struct Plane<V>
{
  center: V,
  normal: V
}

impl<V: Copy> Plane<V>
{
  pub fn new(&center: &V, &normal: &V) -> Plane<V>
  { Plane { center: center, normal: normal } }
}
