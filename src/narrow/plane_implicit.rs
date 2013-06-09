use std::num::Zero;
use nalgebra::traits::dot::{Dot};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::ring::Ring;
use nalgebra::traits::workarounds::scalar_op::ScalarMul;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::contact;
use contact::contact::Contact;

pub fn update_collide_plane_implicit_shape<
         V: VectorSpace<T> + Dot<T> + Copy,
         T: Ring + Ord + Copy,
         G: Implicit<V>,
         C: Contact<V, T>>
   (center: &V, plane: &Plane<V>, other: &G, out: &mut C) -> bool
{
  let deepest = &other.support_point(&-plane.normal);
  let dist    = &plane.normal.dot(&(*center - *deepest));

  if (*dist > Zero::zero())
  {
    let c1 = &(deepest + plane.normal.scalar_mul(dist));
    contact::set(out, c1, deepest, &plane.normal, dist);

    true
  }
  else
  { false }
}

pub fn collide_plane_implicit_shape<
         V: VectorSpace<T> + Dot<T> + Copy,
         T: Ring + Ord + Copy,
         G: Implicit<V>,
         C: Contact<V, T>>
   (center: &V, plane: &Plane<V>, other: &G) -> Option<~C>
{
  let mut res : ~C = ~contact::zero::<V, T, C>();

  if (update_collide_plane_implicit_shape(center, plane, other, res))
  { Some(res) }
  else
  { None }
}
