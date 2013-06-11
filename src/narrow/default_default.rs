use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::norm::Norm;
use geom::default_geom::{DefaultGeom, Ball, Plane};
use geom::ball;
use geom::plane;
use contact::contact::Contact;
use narrow::collision_detector::CollisionDetector;
use narrow::ball_ball::BallBallCollisionDetector;
use narrow::plane_implicit::PlaneImplicitCollisionDetector;

/**
 * Collision detector between two `DefaultGeometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
pub struct DefaultDefaultCollisionDetector<C, N, V>
{
  // FIXME: it would be better to use a ~ instead of a @mut here…
  priv sub_detector: @mut CollisionDetector<C, DefaultGeom<N, V>, DefaultGeom<N, V>>
}

impl<N: Real + Copy,
     C: Contact<V, N> + Copy,
     V: VectorSpace<N> + Norm<N> + Copy> 
CollisionDetector<C, DefaultGeom<N, V>, DefaultGeom<N, V>>
for DefaultDefaultCollisionDetector<C, N, V>

{
  fn new(g1: &DefaultGeom<N, V>, g2: &DefaultGeom<N, V>)
     -> DefaultDefaultCollisionDetector<C, N, V>
  {
    type DG      = DefaultGeom<N, V>;
    type B       = ball::Ball<N, V>;
    type BBCD    = BallBallCollisionDetector<C, N, V>;
    type DW_BBCD = DispatchWrapper<C, N, V, BBCD, B, B>;

    let sub_detector = match (g1, g2)
    {
      (&Ball(_), &Ball(_)) =>
        @mut CollisionDetector::new::<C, DG, DG, DW_BBCD>(g1, g2)
        as @mut CollisionDetector<C, DefaultGeom<N, V>, DefaultGeom<N, V>>,
      (&Ball(_), &Plane(_))  => fail!("bad guy."),
      _ => fail!("Dont know how to dispatch that.")
      // (Ball(_), Plane(_)) =>,
      // (Plane(_), Ball(_)) =>
    };

    DefaultDefaultCollisionDetector {
      sub_detector: sub_detector
    }
  }

  fn update(&mut self, g1: &DefaultGeom<N, V>, g2: &DefaultGeom<N, V>)
  { self.sub_detector.update(g1, g2); }

  fn num_coll(&self) -> uint
  { self.sub_detector.num_coll() }

  fn colls<'a, 'b>(&'a mut self, out_colls: &'b mut ~[&'a mut C])
  { self.sub_detector.colls(out_colls) }
}

// wrappers management
#[doc(hidden)]
trait DefaultExtractor<N, V, G1, G2>
{
  fn extract<'r>(g1: &'r DefaultGeom<N, V>, g2: &'r DefaultGeom<N, V>)
     -> (&'r G1, &'r G2);
}

#[doc(hidden)]
impl<C, N, V>
DefaultExtractor<N, V, ball::Ball<N, V>, ball::Ball<N, V>>
for BallBallCollisionDetector<C, N, V>
{
  fn extract<'r>(g1: &'r DefaultGeom<N, V>, g2: &'r DefaultGeom<N, V>)
     -> (&'r ball::Ball<N, V>, &'r ball::Ball<N, V>)
  { (g1.ball(), g2.ball()) }
}

#[doc(hidden)]
impl<N, V, G, C>
DefaultExtractor<N, V, plane::Plane<V>, ball::Ball<N, V>>
for PlaneImplicitCollisionDetector<N, V, G, C>
{
  fn extract<'r>(g1: &'r DefaultGeom<N, V>, g2: &'r DefaultGeom<N, V>)
     -> (&'r plane::Plane<V>, &'r ball::Ball<N, V>)
  { (g1.plane(), g2.ball()) }
}

#[doc(hidden)]
struct DispatchWrapper<C, N, V, NF, G1, G2>
{ priv sub_detector: ~NF }

#[doc(hidden)]
impl<C,
     N,
     V,
     NF: CollisionDetector<C, G1, G2> + DefaultExtractor<N, V, G1, G2>,
     G1,
     G2>
CollisionDetector<C, DefaultGeom<N, V>, DefaultGeom<N, V>>
for DispatchWrapper<C, N, V, NF, G1, G2>
{
  fn new(g1: &DefaultGeom<N, V>, g2: &DefaultGeom<N, V>)
     -> DispatchWrapper<C, N, V, NF, G1, G2>
  {
    let (a, b): (&G1, &G2) = DefaultExtractor::extract::<N, V, G1, G2, NF>(g1, g2);

    DispatchWrapper {
      sub_detector: ~CollisionDetector::new(a, b)
    }
  }

  fn update(&mut self, g1: &DefaultGeom<N, V>, g2: &DefaultGeom<N, V>)
  {
    let (a, b): (&G1, &G2) = DefaultExtractor::extract::<N, V, G1, G2, NF>(g1, g2);
    self.sub_detector.update(a, b);
  }

  fn num_coll(&self) -> uint
  { self.sub_detector.num_coll() }

  fn colls<'a, 'b>(&'a mut self, out_colls: &'b mut ~[&'a mut C])
  { self.sub_detector.colls(out_colls) }
}
