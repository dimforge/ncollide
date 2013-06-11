use std::vec;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use geom::ball::Ball;
use narrow::collision_detector::CollisionDetector;
use contact::contact;
use contact::contact::Contact;

pub struct BallBallCollisionDetector<C, N, V>
{
  priv contact: Option<C>
}

impl<N: Real + Copy,
     C: Contact<V, N> + Copy,
     V: VectorSpace<N> + Norm<N> + Copy> 
    CollisionDetector<C, Ball<N, V>, Ball<N, V>> for
    BallBallCollisionDetector<C, N, V>
{
 fn new(_: &Ball<N, V>, _: &Ball<N, V>) -> BallBallCollisionDetector<C, N, V>
 { BallBallCollisionDetector{ contact: None } }

 fn update(&mut self, a: &Ball<N, V>, b: &Ball<N, V>)
 {
   if (self.contact.is_none())
   { self.contact = collide_ball_ball(a, b) }
   else
   {
     if !(update_collide_ball_ball(a, b, self.contact.get_mut_ref()))
     { self.contact = None }
   }
 }

 fn num_coll(&self) -> uint
 {
   match self.contact
   {
     None    => 0,
     Some(_) => 1
   }
 }

 fn colls<'a, 'b>(&'a mut self, out_colls: &'b mut ~[&'a mut C])
 {
   match self.contact
   {
     Some(ref mut c) => vec::push(out_colls, c),
     None    => ()
   }
 }
}

pub fn update_collide_ball_ball<V: VectorSpace<N> + Norm<N> + Copy,
                                N: Real + Copy,
                                C: Contact<V, N>>
   (b1: &Ball<N, V>, b2: &Ball<N, V>, out: &mut C) -> bool
{
  let r1         = b1.radius;
  let r2         = b2.radius;
  let delta_pos  = b2.center - b1.center;
  let sqdist     = delta_pos.sqnorm();
  let sum_radius = r1 + r2;

  if (sqdist < sum_radius * sum_radius)
  {
    let normal = delta_pos.normalized();
    contact::set(out,
                 &(b1.center - normal.scalar_mul(&r1)),
                 &(b2.center - normal.scalar_mul(&r2)),
                 &normal,
                 &(sum_radius - sqdist.sqrt()));
    true
  }
  else
  { false }
}

pub fn collide_ball_ball<V: VectorSpace<N> + Norm<N> + Copy,
                         N: Real + Copy,
                         C: Contact<V, N>>
   (b1: &Ball<N, V>, b2: &Ball<N, V>) -> Option<C>
{
  let mut res = contact::zero::<V, N, C>();

  if (update_collide_ball_ball(b1, b2, &mut res))
  { Some(res) }
  else
  { None }
}
