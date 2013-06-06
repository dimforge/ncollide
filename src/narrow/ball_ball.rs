use std::num::Zero;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::vector_space::VectorSpace;
use geom::transformed::Transformed;
use geom::ball::Ball;
use contact::Contact;


fn update_collide_ball_ball<V: VectorSpace<T> + Norm<T> + Copy, T: Real + Copy>
   (c1: &V, b1: &Ball<T>, c2: &V, b2: &Ball<T>, out: &mut Contact<V, T>) -> bool
{
  let r1         = b1.radius;
  let r2         = b2.radius;
  let delta_pos  = *c2 - *c1;
  let sqdist     = delta_pos.sqnorm();
  let sum_radius = r1 + r2;

  if (sqdist < sum_radius * sum_radius)
  {
    let normal = delta_pos.normalized();
    out.set(&(c1 - normal.scalar_mul(&r1)),
            &(c2 - normal.scalar_mul(&r2)),
            &normal,
            &(sum_radius - sqdist.sqrt()));
    true
  }
  else
  { false }
}

fn collide_ball_ball<V: VectorSpace<T> + Norm<T> + Copy, T: Real + Copy>
   (c1: &V, b1: &Ball<T>, c2: &V, b2: &Ball<T>) -> Option<~Contact<V, T>>
{
  let mut res = 
    ~Contact { world_contact1: Zero::zero(),
               world_contact2: Zero::zero(),
               center:         Zero::zero(),
               normal:         Zero::zero(),
               depth:          Zero::zero() };

  if (update_collide_ball_ball(c1, b1, c2, b2, res))
  { Some(res) }
  else
  { None }
}

fn update_collide_tball_tball
   <T: Translation<V>,
    V: VectorSpace<N> + Norm<N> + Copy,
    N: Real + Copy>
   (b1:  &Transformed<Ball<N>, T>,
    b2:  &Transformed<Ball<N>, T>,
    out: &mut Contact<V, N>) -> bool
{
  update_collide_ball_ball(&b1.t.translation(), b1.g,
                           &b2.t.translation(), b2.g,
                           out)
}

fn collide_tball_tball
   <T: Translation<V>,
    V: VectorSpace<N> + Norm<N> + Copy,
    N: Real + Copy>
   (b1:  &Transformed<Ball<N>, T>,
    b2:  &Transformed<Ball<N>, T>) -> Option<~Contact<V, N>>
{
  collide_ball_ball(&b1.t.translation(), b1.g, &b2.t.translation(), b2.g)
}
