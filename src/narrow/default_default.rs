use std::rand::Rand;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::norm::Norm;
use nalgebra::traits::division_ring::DivisionRing;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::dim::Dim;
use nalgebra::traits::transformation::Transform;
use nalgebra::traits::sample::UniformSphereSample;
use geom::default_geom::{DefaultGeom, Ball, Plane, Implicit};
use geom::implicit::Implicit;
use geom::ball::Ball;
use geom::minkowski_sum::AnnotatedPoint;
use contact::contact::UpdatableContact;
use narrow::algorithm::simplex::Simplex;
use narrow::collision_detector::CollisionDetector;
use narrow::implicit_implicit::ImplicitImplicitCollisionDetector;
use narrow::ball_ball::BallBallCollisionDetector;
use ICMG = narrow::incremental_contact_manifold_generator::IncrementalContactManifoldGenerator;
use narrow::plane_implicit::{PlaneImplicitCollisionDetector,
                             ImplicitPlaneCollisionDetector};

enum DefaultDefaultCollisionDetector<C, N, V, M, S, I>
{
  BallBall        (BallBallCollisionDetector<C, N, V>),
  BallPlane       (ImplicitPlaneCollisionDetector<N, V, Ball<N, V>, C>),
  PlaneBall       (PlaneImplicitCollisionDetector<N, V, Ball<N, V>, C>),
  BallImplicit    (ImplicitImplicitCollisionDetector<S, C, Ball<N, V>, I, V, N>),
  ImplicitBall    (ImplicitImplicitCollisionDetector<S, C, I, Ball<N, V>, V, N>),
  PlaneImplicit   (ICMG<C, PlaneImplicitCollisionDetector<N, V, I, C>, V, N>),
  ImplicitPlane   (ICMG<C, ImplicitPlaneCollisionDetector<N, V, I, C>, V, N>),
  ImplicitImplicit(ICMG<C, ImplicitImplicitCollisionDetector<S, C, I, I, V, N>, V, N>)
}

/**
 * Collision detector between two `DefaultGeometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
// FIXME: Ring + Real ?
impl<N: ApproxEq<N> + DivisionRing + Real + Float + Ord + Clone,
     C: UpdatableContact<V, N> + Clone + ToStr + Freeze + DeepClone,
     V: VectorSpace<N> + Dim + Dot<N> + Norm<N> + Rand + UniformSphereSample + ApproxEq<N> + Eq + Clone,
     M,
     S: Simplex<AnnotatedPoint<V>, N>,
     I: Implicit<V> + Transform<V>>
CollisionDetector<C, DefaultGeom<N, V, M, I>, DefaultGeom<N, V, M, I>>
for DefaultDefaultCollisionDetector<C, N, V, M, S, I>

{
  fn new(g1: &DefaultGeom<N, V, M, I>, g2: &DefaultGeom<N, V, M, I>)
     -> DefaultDefaultCollisionDetector<C, N, V, M, S, I>
  {
    match (g1, g2)
    {
      (&Ball(_), &Ball(_))         => BallBall(CollisionDetector::new(g1.ball(), g2.ball())),
      (&Ball(_), &Plane(_))        => BallPlane(CollisionDetector::new(g1.ball(), g2.plane())),
      (&Plane(_), &Ball(_))        => PlaneBall(CollisionDetector::new(g1.plane(), g2.ball())),
      (&Implicit(_), &Ball(_))     => ImplicitBall(CollisionDetector::new(g1.implicit(), g2.ball())),
      (&Ball(_), &Implicit(_))     => BallImplicit(CollisionDetector::new(g1.ball(), g2.implicit())),
      (&Implicit(_), &Plane(_))    => ImplicitPlane(CollisionDetector::new(g1.implicit(), g2.plane())),
      (&Plane(_), &Implicit(_))    => PlaneImplicit(CollisionDetector::new(g1.plane(), g2.implicit())),
      (&Implicit(_), &Implicit(_)) => ImplicitImplicit(CollisionDetector::new(g1.implicit(), g2.implicit())),
      _ => fail!("Dont know how to dispatch that.")
    }
  }

  #[inline]
  fn update(&mut self, g1: &DefaultGeom<N, V, M, I>, g2: &DefaultGeom<N, V, M, I>)
  {
    match *self
    {
      BallBall        (ref mut cd) => cd.update(g1.ball(),     g2.ball()),
      BallPlane       (ref mut cd) => cd.update(g1.ball(),     g2.plane()),
      PlaneBall       (ref mut cd) => cd.update(g1.plane(),    g2.ball()),
      BallImplicit    (ref mut cd) => cd.update(g1.ball(),     g2.implicit()),
      ImplicitBall    (ref mut cd) => cd.update(g1.implicit(), g2.ball()),
      PlaneImplicit   (ref mut cd) => cd.update(g1.plane(),    g2.implicit()),
      ImplicitPlane   (ref mut cd) => cd.update(g1.implicit(), g2.plane()),
      ImplicitImplicit(ref mut cd) => cd.update(g1.implicit(), g2.implicit())
    }
  }

  #[inline]
  fn num_coll(&self) -> uint
  {
    match *self
    {
      BallBall        (ref cd) => cd.num_coll(),
      BallPlane       (ref cd) => cd.num_coll(),
      PlaneBall       (ref cd) => cd.num_coll(),
      BallImplicit    (ref cd) => cd.num_coll(),
      ImplicitBall    (ref cd) => cd.num_coll(),
      PlaneImplicit   (ref cd) => cd.num_coll(),
      ImplicitPlane   (ref cd) => cd.num_coll(),
      ImplicitImplicit(ref cd) => cd.num_coll()
    }
  }

  #[inline]
  fn colls(&mut self, out_colls: &mut ~[@mut C])
  {
    match *self
    {
      BallBall        (ref mut cd) => cd.colls(out_colls),
      BallPlane       (ref mut cd) => cd.colls(out_colls),
      PlaneBall       (ref mut cd) => cd.colls(out_colls),
      BallImplicit    (ref mut cd) => cd.colls(out_colls),
      ImplicitBall    (ref mut cd) => cd.colls(out_colls),
      PlaneImplicit   (ref mut cd) => cd.colls(out_colls),
      ImplicitPlane   (ref mut cd) => cd.colls(out_colls),
      ImplicitImplicit(ref mut cd) => cd.colls(out_colls)
    }
  }
}
