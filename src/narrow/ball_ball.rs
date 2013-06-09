use nalgebra::traits::translation::Translation;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use geom::transformed::Transformed;
use geom::ball::Ball;
use contact::contact;
use contact::contact::Contact;

pub fn update_collide_ball_ball<V: VectorSpace<T> + Norm<T> + Copy,
                                T: Real + Copy,
                                C: Contact<V, T>>
   (c1: &V, b1: &Ball<T>, c2: &V, b2: &Ball<T>, out: &mut C) -> bool
{
  let r1         = b1.radius;
  let r2         = b2.radius;
  let delta_pos  = *c2 - *c1;
  let sqdist     = delta_pos.sqnorm();
  let sum_radius = r1 + r2;

  if (sqdist < sum_radius * sum_radius)
  {
    let normal = delta_pos.normalized();
    contact::set(out,
                 &(c1 - normal.scalar_mul(&r1)),
                 &(c2 - normal.scalar_mul(&r2)),
                 &normal,
                 &(sum_radius - sqdist.sqrt()));
    true
  }
  else
  { false }
}

pub fn collide_ball_ball<V: VectorSpace<T> + Norm<T> + Copy,
                         T: Real + Copy,
                         C: Contact<V, T>>
   (c1: &V, b1: &Ball<T>, c2: &V, b2: &Ball<T>) -> Option<~C>
{
  let mut res = ~contact::zero::<V, T, C>();

  if (update_collide_ball_ball(c1, b1, c2, b2, res))
  { Some(res) }
  else
  { None }
}

pub fn update_collide_tball_tball<T: Translation<V>,
                                  V: VectorSpace<N> + Norm<N> + Copy,
                                  N: Real + Copy,
                                  C: Contact<V, N>>
       (b1:  &Transformed<Ball<N>, T>,
        b2:  &Transformed<Ball<N>, T>,
        out: &mut C) -> bool
{
  update_collide_ball_ball(&b1.t.translation(), b1.g,
                           &b2.t.translation(), b2.g,
                           out)
}

pub fn collide_tball_tball<T: Translation<V>,
                           V: VectorSpace<N> + Norm<N> + Copy,
                           N: Real + Copy,
                           C: Contact<V, N>>
   (b1:  &Transformed<Ball<N>, T>, b2:  &Transformed<Ball<N>, T>) -> Option<~C>
{
  collide_ball_ball(&b1.t.translation(), b1.g, &b2.t.translation(), b2.g)
}
