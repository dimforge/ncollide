pub trait Exact<A>
{
  fn exact<'r>(&'r self) -> &'r A;
  fn exact_mut<'r>(&'r mut self) -> &'r mut A;
}

impl<A> Exact<A> for A
{
  fn exact<'r>(&'r self) -> &'r A
  { self }

  fn exact_mut<'r>(&'r mut self) -> &'r mut A
  { self }
}
