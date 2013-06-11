use narrow::collision_detector::CollisionDetector;

pub struct EmptyDetector;

impl<C, G1, G2> CollisionDetector<C, G1, G2> for EmptyDetector
{
  fn new(_: &G1, _: &G2) -> EmptyDetector
  { EmptyDetector }

  fn update(&mut self, _: &G1, _: &G2)
  { }

  fn num_coll(&self) -> uint
  { 0u }

  fn colls<'a, 'b>(&'a mut self, _: &'b mut ~[&'a mut C])
  { }
}
