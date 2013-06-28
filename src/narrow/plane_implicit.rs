use std::num::Zero;
use nalgebra::traits::dot::{Dot};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::ring::Ring;
use nalgebra::traits::scalar_op::ScalarMul;
use narrow::collision_detector::CollisionDetector;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::contact;
use contact::contact::Contact;

/**
 * Collision detector between a plane and a geometry implementing the
 * `Implicit` trait.
 */
pub struct PlaneImplicitCollisionDetector<N, V, G, C>
{ priv contact: Option<@mut C> }

pub struct ImplicitPlaneCollisionDetector<N, V, G, C>
{ priv contact: Option<@mut C> }

impl<V: VectorSpace<N> + Dot<N> + Copy,
     N: Ring + Ord + Copy,
     G: Implicit<V>,
     C: Contact<V, N>>
    CollisionDetector<C, Plane<V>, G> for PlaneImplicitCollisionDetector<N, V, G, C>
{
  #[inline]
 fn new(_: &Plane<V>, _: &G) -> PlaneImplicitCollisionDetector<N, V, G, C>
 { PlaneImplicitCollisionDetector{ contact: None } }

 fn update(&mut self, a: &Plane<V>, b: &G)
 {
   match self.contact
   {
     None => self.contact = collide_plane_implicit_shape(a, b).map(|&c| @mut c),
     Some(c) => if !update_collide_plane_implicit_shape(a, b, c)
                { self.contact = None }
   }
 }

  #[inline]
 fn num_coll(&self) -> uint
 {
   match self.contact
   {
     None    => 0,
     Some(_) => 1
   }
 }

  #[inline]
 fn colls(&mut self, out_colls: &mut ~[@mut C])
 {
   match self.contact
   {
     Some(c) => out_colls.push(c),
     None    => ()
   }
 }
}

impl<V: VectorSpace<N> + Dot<N> + Copy,
     N: Ring + Ord + Copy,
     G: Implicit<V>,
     C: Contact<V, N>>
    CollisionDetector<C, G, Plane<V>> for ImplicitPlaneCollisionDetector<N, V, G, C>
{
  #[inline]
 fn new(_: &G, _: &Plane<V>) -> ImplicitPlaneCollisionDetector<N, V, G, C>
 { ImplicitPlaneCollisionDetector{ contact: None } }

 fn update(&mut self, a: &G, b: &Plane<V>)
 {
   match self.contact
   {
     None => self.contact = collide_plane_implicit_shape(b, a).map(|&c| @mut c),
     Some(c) => if !update_collide_plane_implicit_shape(b, a, c)
                { self.contact = None }
   }

   self.contact = self.contact.map(|&c| { c.flip(); c })
 }

  #[inline]
 fn num_coll(&self) -> uint
 {
   match self.contact
   {
     None    => 0,
     Some(_) => 1
   }
 }

  #[inline]
 fn colls(&mut self, out_colls: &mut ~[@mut C])
 {
   match self.contact
   {
     Some(c) => out_colls.push(c),
     None    => ()
   }
 }
}

/**
 * Computes the collision between a plane and an implicit geometry. Returns
 * whether they are colliding.
 *
 *   - `plane`: the plane to test.
 *   - `other`: the object to test against the plane.
 *   - `out`: collision on which the result will be written.
 */
pub fn update_collide_plane_implicit_shape<V: VectorSpace<N> + Dot<N> + Copy,
                                           N: Ring + Ord + Copy,
                                           G: Implicit<V>,
                                           C: Contact<V, N>>
   (plane: &Plane<V>, other: &G, out: &mut C) -> bool
{
  let deepest = other.support_point(&-plane.normal());
  let dist    = plane.normal().dot(&(plane.center() - deepest));

  if dist > Zero::zero()
  {
    let c1 = deepest + plane.normal().scalar_mul(&dist);
    contact::set(out, c1, deepest, plane.normal(), dist);

    true
  }
  else
  { false }
}

/**
 * Same as `update_collide_plane_implicit_shape` but the existing collision or
 * `None`.
 *
 *   - `plane`: the plane to test.
 *   - `other`: the object to test against the plane.
 */
pub fn collide_plane_implicit_shape<V: VectorSpace<N> + Dot<N> + Copy,
                                    N: Ring + Ord + Copy,
                                    G: Implicit<V>,
                                    C: Contact<V, N>>
   (plane: &Plane<V>, other: &G) -> Option<C>
{
  let mut res : C = contact::zero::<V, N, C>();

  if update_collide_plane_implicit_shape(plane, other, &mut res)
  { Some(res) }
  else
  { None }
}
