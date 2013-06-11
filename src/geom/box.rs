#[deriving(Eq, ToStr)]
pub struct Box<D, N>
{
  priv components: ~[N]
}

// FIXME: implementing support_point needs some more traits:
// Functor (to map a function of the vector)
// Pointwise to to component-wise operations.
