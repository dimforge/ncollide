use geom::ball;
use geom::plane;

pub enum DefaultGeom<N, V> {
  Plane(plane::Plane<V>),
  Ball(ball::Ball<N, V>)
}

impl<N, V> DefaultGeom<N, V>
{
  pub fn ball<'r>(&'r self) -> &'r ball::Ball<N, V>
  {
    match *self {
        Ball(ref b) => b,
        _ => fail!("Unexpected geometry: this is not a ball.")
    }
  }

  pub fn plane<'r>(&'r self) -> &'r plane::Plane<V>
  {
    match *self {
        Plane(ref p) => p,
        _ => fail!("Unexpected geometry: this is not a plane.")
    }
  }
}
