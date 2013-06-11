use geom::ball;
use geom::plane;

/**
 * Enumeration grouping all common shapes. Used to simplify collision detection
 * dispatch.
 */
#[deriving(Eq, ToStr)]
pub enum DefaultGeom<N, V> {
  Plane(plane::Plane<V>),
  Ball(ball::Ball<N, V>)
}

impl<N, V> DefaultGeom<N, V>
{
  /**
   * Convenience method to extract a ball from the enumation. Fails if the
   * pattern `Ball` is not matched.
   */
  pub fn ball<'r>(&'r self) -> &'r ball::Ball<N, V>
  {
    match *self {
        Ball(ref b) => b,
        _ => fail!("Unexpected geometry: this is not a ball.")
    }
  }

  /**
   * Convenience method to extract a plane from the enumation. Fails if the
   * pattern `Plane` is not matched.
   */
  pub fn plane<'r>(&'r self) -> &'r plane::Plane<V>
  {
    match *self {
        Plane(ref p) => p,
        _ => fail!("Unexpected geometry: this is not a plane.")
    }
  }
}
