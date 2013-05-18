#[deriving(Eq)]
pub struct Box<D, T>
{
  components: ~[T]
}

// FIXME: implementing support_point needs some more traits:
// Functor (to map a function of the vector)
// Pointwise to to component-wise operations.
