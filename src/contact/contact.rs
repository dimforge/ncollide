use std::num::{One, Zero};
use nalgebra::traits::scalar_op::ScalarDiv;

/// Trait of contacts. A contact is the structure describing the geometric
/// configurations of the contact point of two touching/penetrating objects.
pub trait Contact<V, N>
{
  /// Creates a new contact from its center, normal, depth, and exact contact
  /// points.
  fn new(center: V, normal: V, depth: N, world1: V, world2: V) -> Self;

  /// Flips the contact. The normal should be inverted and both exact contact
  /// points should be swaped.
  fn flip(&mut self);

  /// Sets the contact center. This is the approximation of the contact point.
  fn set_center(&mut self, V);
  /// The contact center.
  fn center(&self) -> V;

  /// Sets the contact normal.
  fn set_normal(&mut self, V);
  /// The contact normal.
  fn normal(&self) -> V;

  /// Sets the penetration depth along the contact normal.
  fn set_depth(&mut self, N);
  /// The penetration depth along the contact normal.
  fn depth(&self) -> N;

  /// Sets the exact contact point on the first object involved.
  fn set_world1(&mut self, V);
  /// The exact contact point on the first object involved.
  fn world1(&self) -> V;

  /// Sets the exact contact point on the second object involved.
  fn set_world2(&mut self, V);
  /// The exact contact point on the second object involved.
  fn world2(&self) -> V;
}

pub trait UpdatableContact<V, N> : Contact<V, N>
{
  fn set_local1(&mut self, V);
  fn local1(&self) -> V;

  fn set_local2(&mut self, V);
  fn local2(&self) -> V;
}

/**
 * Creates a new, meaninless contact.
 */
pub fn zero<V: Zero, N: Zero, C: Contact<V, N>>() -> C
{
  Contact::new(Zero::zero::<V>(),
               Zero::zero::<V>(),
               Zero::zero::<N>(),
               Zero::zero::<V>(),
               Zero::zero::<V>())
}

/**
 * Sets all the fields of a contact. The contact center is automatically
 * approximated as the center of the two exact contact points.
 *
 *   - `contact`: the contact being modified.
 *   - `wc1`: the exact contact point on the first object involved.
 *   - `wc2`: the exact contact point on the second object involved.
 *   - `normal`: the contact normal.
 *   - `depth`: the penetration depth along the normal.
 */
pub fn set<V: Add<V, V> + ScalarDiv<N>,
           N: One + Add<N, N>,
           C: Contact<V, N>>
       (contact: &mut C, wc1: V, wc2: V, normal: V, depth: N)
{
  let _2 = &(One::one::<N>() + One::one()); // FIXME: a better way to do that?

  contact.set_center((wc1 + wc2).scalar_div(_2));
  contact.set_world1(wc1);
  contact.set_world2(wc2);
  contact.set_normal(normal);
  contact.set_depth (depth);
}

pub fn copy_to<V: Add<V, V> + ScalarDiv<N>,
               N: One + Add<N, N>,
               C: Contact<V, N>>
       (in: &C, out: &mut C)
{
  out.set_center(in.center());
  out.set_world1(in.world1());
  out.set_world2(in.world2());
  out.set_normal(in.normal());
  out.set_depth(in.depth());
}

pub fn copy_updatable_to<V: Add<V, V> + ScalarDiv<N>,
                         N: One + Add<N, N>,
                         C: UpdatableContact<V, N>>
       (in: &C, out: &mut C)
{
  out.set_center(in.center());
  out.set_world1(in.world1());
  out.set_world2(in.world2());
  out.set_normal(in.normal());
  out.set_depth(in.depth());
  out.set_local1(in.local1());
  out.set_local2(in.local2());
}
