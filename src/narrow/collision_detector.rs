pub trait CollisionDetector<C, G1, G2>
{
  fn new(&G1, &G2) -> Self;
  fn update(&mut self, &G1, &G2);
  fn num_coll(&self) -> uint;
  fn colls<'a, 'b>(&'a mut self, &'b mut ~[&'a mut C]);
}
