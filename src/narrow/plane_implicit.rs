use core::num::{Zero, One};
use nalgebra::traits::dot::{Dot};
use nalgebra::traits::workarounds::scalar_op::{ScalarMul, ScalarDiv};
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;

pub fn update_collide_plane_implicit_shape<
         T: Zero + One + Ord + Add<T, T> + Copy,
         V: Dot<T> + Neg<V> + Add<V, V> + ScalarMul<T> + ScalarDiv<T> + Copy,
         G: Implicit<V>>
   (plane: Plane<V>, other: G, out: &mut Contact<V, T>) -> bool
{
  let deepest = &other.support_point(&-plane.normal);
  let dist    = &plane.normal.dot(&-deepest);     // FIXME: handle the plane center

  if (*dist > Zero::zero())
  {
    let c1 = &(deepest + plane.normal.scalar_mul(dist));
    out.set(c1, deepest, &plane.normal, dist);

    true
  }
  else
  { false }
}

pub fn collide_plane_implicit_shape<
         T: Zero + One + Ord + Add<T, T> + Copy,
         V: Zero + Dot<T> + Neg<V> + Add<V, V> + ScalarMul<T> + ScalarDiv<T> + Copy,
         G: Implicit<V>>
   (plane: Plane<V>, other: G) -> Option<~Contact<V, T>>
{
  let mut res = 
    ~Contact { world_contact1: Zero::zero(),
               world_contact2: Zero::zero(),
               center:         Zero::zero(),
               normal:         Zero::zero(),
               depth:          Zero::zero() };

  if (update_collide_plane_implicit_shape(plane, other, res))
  { Some(res) }
  else
  { None }
}
