use std::num::Zero;
use nalgebra::traits::dot::{Dot};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::ring::Ring;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;

pub fn update_collide_plane_implicit_shape<
         V: VectorSpace<T> + Dot<T> + Copy,
         T: Ring + Ord + Copy,
         G: Implicit<V>>
   (center: &V, plane: &Plane<V>, other: &G, out: &mut Contact<V, T>) -> bool
{
  let deepest = &other.support_point(&-plane.normal);
  let dist    = &plane.normal.dot(&(*center - *deepest));

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
         V: VectorSpace<T> + Dot<T> + Copy,
         T: Ring + Ord + Copy,
         G: Implicit<V>>
   (center: &V, plane: &Plane<V>, other: &G) -> Option<~Contact<V, T>>
{
  let mut res = 
    ~Contact { world_contact1: Zero::zero(),
               world_contact2: Zero::zero(),
               center:         Zero::zero(),
               normal:         Zero::zero(),
               depth:          Zero::zero() };

  if (update_collide_plane_implicit_shape(center, plane, other, res))
  { Some(res) }
  else
  { None }
}
