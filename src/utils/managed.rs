use std::managed;

pub fn position_elem_mut_ptr<RB>(l: &[@mut RB], e: @mut RB) -> Option<uint>
{
  for l.iter().enumerate().advance |(i, &curr)|
  {
    if managed::mut_ptr_eq(e, curr)
    { return Some(i) }
  }

  None
}
