pub trait Simplex<V, N>
{
  fn new(V)                               -> Self;
  fn add_point(&mut self, V);
  fn project_origin_and_reduce(&mut self) -> V;
  fn project_origin(&mut self)            -> V;
  fn contains_point(&self, &V)            -> bool; // FIXME: remove that
  fn dimension(&self)                     -> uint;
  fn max_sq_len(&self)                    -> N;
}
