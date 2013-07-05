use std::uint;
use std::num::{One, Zero};
use nalgebra::traits::dim::Dim;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::transformation::Transform;
use narrow::collision_detector::CollisionDetector;
use contact::contact::{Contact, UpdatableContact};
use contact::contact;

pub struct IncrementalContactManifoldGenerator<C, CD, V, N>
{
  priv contacts:     ~[@mut C],
  priv collector:    ~[@mut C],
  priv sub_detector: CD
}

impl<C:  UpdatableContact<V, N> + ToStr + Clone + Freeze + DeepClone,
     CD: CollisionDetector<C, G1, G2>,
     G1: Transform<V>,
     G2: Transform<V>,
     V: VectorSpace<N> + Dot<N> + Norm<N> + ApproxEq<N> + Dim,
     N: DivisionRing + Ord + NumCast>
CollisionDetector<C, G1, G2> for IncrementalContactManifoldGenerator<C, CD, V, N>
{
  fn new(g1: &G1, g2: &G2) -> IncrementalContactManifoldGenerator<C, CD, V, N>
  {
    IncrementalContactManifoldGenerator {
      contacts:     ~[],
      collector:    ~[],
      sub_detector: CollisionDetector::new(g1, g2)
    }
  }

  #[inline]
  fn update(&mut self, g1: &G1, g2: &G2)
  {
    // cleanup existing contacts
    let mut i = 0;
    while i != self.contacts.len()
    {
      let c      = self.contacts[i];
      let _2     = One::one::<N>() + One::one();
      let world1 = g1.transform_vec(&c.local1());
      let world2 = g2.transform_vec(&c.local2());

      let dw    = world1 - world2;
      let depth = dw.dot(&c.normal());

      let _tangencial_limit: N = NumCast::from(0.25f64);

      if depth < Zero::zero() ||
         (dw - c.normal().scalar_mul(&depth)).sqnorm() > _tangencial_limit * depth * depth
      { self.contacts.swap_remove(i); }
      else
      {
        c.set_depth(depth);

        c.set_center((world1 + world2).scalar_div(&_2));
        c.set_world1(world1);
        c.set_world2(world2);

        i = i + 1;
      }
    }

    // add the new ones
    self.sub_detector.update(g1, g2);

    self.sub_detector.colls(&mut self.collector);

    // compute the local-space contacts
    for self.collector.iter().advance |c|
    {
      let local1 = g1.inv_transform(&c.world1());
      let local2 = g2.inv_transform(&c.world2());

      c.set_local1(local1);
      c.set_local2(local2);
    }

    // remove duplicates
    let _max_num_contact = (Dim::dim::<V>() - 1) * 2;

    for self.collector.iter().advance |c|
    {
      if self.contacts.len() == _max_num_contact
      { add_reduce_by_variance(self.contacts, *c) }
      else
      { self.contacts.push(c.deep_clone()) }
    }

    self.collector.clear();
  }

  #[inline]
  fn num_coll(&self) -> uint
  { self.contacts.len() }

  #[inline]
  fn colls(&mut self, out_colls: &mut ~[@mut C])
  {
    for self.contacts.iter().advance() |c|
    { out_colls.push(*c) }
  }
}

fn add_reduce_by_variance<C: UpdatableContact<V, N>,
                          N: DivisionRing + NumCast + Ord,
                          V: VectorSpace<N> + Norm<N>>(pts: &[@mut C], to_add: &C)
{
  let mut argmax = 0;
  let mut varmax = approx_variance(pts, to_add, 0);

  for uint::iterate(1u, pts.len()) |i|
  {
    let var = approx_variance(pts, to_add, i);

    if var > varmax
    {
      argmax = i;
      varmax = var;
    }
  }

  contact::copy_updatable_to(to_add, pts[argmax])
}

fn approx_variance<C: Contact<V, N>,
                   N: DivisionRing + NumCast,
                   V: VectorSpace<N> + Norm<N>>(pts: &[@mut C], to_add: &C, to_ignore: uint) -> N
{
  // first: compute the mean
  let mut mean = to_add.center();

  for uint::iterate(0u, pts.len()) |i|
  {
    if i != to_ignore
    { mean = mean + pts[i].center() }
  }

  mean.scalar_div_inplace(&NumCast::from(pts.len()));

  // compute the sum of variances along all axis
  let mut sum = (to_add.center() - mean).sqnorm();

  for uint::iterate(0u, pts.len()) |i|
  {
    if i != to_ignore
    { sum = sum + (pts[i].center() - mean).sqnorm(); }
  }

  sum
}
