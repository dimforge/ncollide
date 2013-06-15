// FIXME: this is weird there is no traits like that on std
pub trait Default
{
  fn default() -> Self;
}
