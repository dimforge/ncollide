use std::num::{Zero, One};
use nalgebra::na::{Cast, Translation, Rotate, Rotation, AbsoluteRotate, Transform,
                   Inv, Vec, AlgebraicVecExt, Cross};
use geom::{Ball, AnnotatedPoint, CompoundAABB};
use geom::{Geom, IGeom, PlaneGeom, BallGeom, ImplicitGeom, CompoundGeom};
use contact::Contact;
use broad;
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use narrow::toi;
use narrow::{CollisionDetector, ImplicitImplicit, BallBall, AnyCompoundAABB,
             ImplicitPlane, PlaneImplicit, CompoundAABBAny, CompoundAABBCompoundAABB};
use OSCMG = narrow::OneShotContactManifoldGenerator;

type S<N, LV> = JohnsonSimplex<N, AnnotatedPoint<LV>>;
type C<N, LV, M> = CompoundAABB<N, LV, M, Geom<N, LV, M>>;
type CA<N, LV, AV, M> = CompoundAABBAny<N, LV, M,
                                        Geom<N, LV, M>,
                                        Dispatcher<N, LV, AV, M>,
                                        GeomGeom<N, LV, AV, M>>;
type AC<N, LV, AV, M> = AnyCompoundAABB<N, LV, M,
                                        Geom<N, LV, M>,
                                        Dispatcher<N, LV, AV, M>,
                                        GeomGeom<N, LV, AV, M>>;

/// Enum grouping collision detectors.
pub enum GeomGeom<N, LV, AV, M> {
    /// Ball vs. ball collision detector.
    BallBall(BallBall<N, LV, M>),
    /// Ball vs. plane collision detector.
    BallPlane(ImplicitPlane<N, LV, M, Ball<N>>),
    /// Plane vs. ball collision detector.
    PlaneBall(PlaneImplicit<N, LV, M, Ball<N>>),
    /// Ball vs. implicit geometry collision detector.
    BallImplicit(ImplicitImplicit<S<N, LV>, Ball<N>, IGeom<N, LV, M>, N, LV>),
    /// Implicit geometry vs. ball collision detector.
    ImplicitBall(ImplicitImplicit<S<N, LV>, IGeom<N, LV, M>, Ball<N>, N, LV>),
    /// Plane vs. implicit geometry collision detector.
    PlaneImplicit(OSCMG<PlaneImplicit<N, LV, M, IGeom<N, LV, M>>, N, LV, AV, M>),
    /// Implicit geometry vs. plane collision detector.
    ImplicitPlane(OSCMG<ImplicitPlane<N, LV, M, IGeom<N, LV, M>>, N, LV, AV, M>),
    /// Implicit geometry vs. implicit geometry collision detector.
    ImplicitImplicit(OSCMG<ImplicitImplicit<S<N, LV>, IGeom<N, LV, M>, IGeom<N, LV, M>, N, LV>, N, LV, AV, M>),
    /// Compound geometry vs. compound geometry collision detector.
    CompoundCompound(CompoundAABBCompoundAABB<N, LV, M,
                                              Geom<N, LV, M>,
                                              Dispatcher<N, LV, AV, M>,
                                              GeomGeom<N, LV, AV, M>>),
    /// Compound geometry vs. any other geometry collision detector.
    CompoundAny(CA<N, LV, AV, M>),
    /// Any geometry vs. compound geometry collision detector.
    AnyCompound(AC<N, LV, AV, M>)
}

impl<N:  Send + Freeze + Zero + Cast<f32> + Clone,
     LV: Send + Freeze + Clone,
     AV,
     M: Send + Freeze>
GeomGeom<N, LV, AV, M> {
    /// Creates a new Geom vs Geom collision detector.
    pub fn new(g1:     &Geom<N, LV, M>,
               g2:     &Geom<N, LV, M>,
               s:      &JohnsonSimplex<N, AnnotatedPoint<LV>>)
               -> GeomGeom<N, LV, AV, M> {
        match (g1, g2) {
            (&ImplicitGeom(BallGeom(_)), &ImplicitGeom(BallGeom(_))) => {
                BallBall(BallBall::new(Cast::from(0.1)))
            },
            (&ImplicitGeom(BallGeom(_)), &PlaneGeom(_)) => {
                BallPlane(ImplicitPlane::new(Cast::from(0.1)))
            },
            (&PlaneGeom(_), &ImplicitGeom(BallGeom(_))) => {
                PlaneBall(PlaneImplicit::new(Cast::from(0.1)))
            },
            (&ImplicitGeom(_), &ImplicitGeom(BallGeom(_))) => {
                ImplicitBall(ImplicitImplicit::new(Cast::from(0.1), s.clone()))
            },
            (&ImplicitGeom(BallGeom(_)), &ImplicitGeom(_)) => {
                BallImplicit(ImplicitImplicit::new(Cast::from(0.1), s.clone()))
            },
            (&ImplicitGeom(_), &PlaneGeom(_)) => {
                ImplicitPlane(OSCMG::new(Cast::from(0.1), ImplicitPlane::new(Zero::zero())))
            },
            (&PlaneGeom(_), &ImplicitGeom(_)) => {
                PlaneImplicit(OSCMG::new(Cast::from(0.1), PlaneImplicit::new(Zero::zero())))
            },
            (&ImplicitGeom(_), &ImplicitGeom(_)) => {
                ImplicitImplicit(
                OSCMG::new(Cast::from(0.1), ImplicitImplicit::new(Zero::zero(), s.clone())))
            },
            (&CompoundGeom(ref c1), &CompoundGeom(ref c2)) => {
                CompoundCompound(CompoundAABBCompoundAABB::new(Dispatcher::new(s.clone()), c1.get(), c2.get()))
            },
            (&CompoundGeom(ref c), _) => {
                CompoundAny(CompoundAABBAny::new(Dispatcher::new(s.clone()), c.get()))
            },
            (_, &CompoundGeom(ref c)) => {
                AnyCompound(AnyCompoundAABB::new(Dispatcher::new(s.clone()), c.get()))
            },
            _ => fail!("Dont know how to dispatch that.")
        }
    }
}

/**
 * Collision detector between two `Geometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
impl<N:  Send + Freeze + ApproxEq<N> + Num + Real + Float + Ord + Cast<f32> + Clone + Algebraic,
     LV: 'static + Send + Freeze + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone +
         Rotate<LV> + Transform<LV>,
     AV: Vec<N>,
     M:  Send + Freeze + Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> +
         AbsoluteRotate<LV> + Mul<M, M> + Inv + One>
CollisionDetector<N, LV, M, Geom<N, LV, M>, Geom<N, LV, M>>
for GeomGeom<N, LV, AV, M> {
    #[inline]
    fn update(&mut self,
              m1: &M,
              g1: &Geom<N, LV, M>,
              m2: &M,
              g2: &Geom<N, LV, M>) {
        match *self {
            BallBall(ref mut cd)         => cd.update(m1, g1.ball(),     m2, g2.ball()),
            BallPlane(ref mut cd)        => cd.update(m1, g1.ball(),     m2, g2.plane()),
            PlaneBall(ref mut cd)        => cd.update(m1, g1.plane(),    m2, g2.ball()),
            BallImplicit(ref mut cd)     => cd.update(m1, g1.ball(),     m2, g2.implicit()),
            ImplicitBall(ref mut cd)     => cd.update(m1, g1.implicit(), m2, g2.ball()),
            PlaneImplicit(ref mut cd)    => cd.update(m1, g1.plane(),    m2, g2.implicit()),
            ImplicitPlane(ref mut cd)    => cd.update(m1, g1.implicit(), m2, g2.plane()),
            ImplicitImplicit(ref mut cd) => cd.update(m1, g1.implicit(), m2, g2.implicit()),
            CompoundCompound(ref mut cd) => cd.update(m1, g1.compound(), m2, g2.compound()),
            CompoundAny(ref mut cd)      => cd.update(m1, g1.compound(), m2, g2),
            AnyCompound(ref mut cd)      => cd.update(m1, g1,            m2, g2.compound())
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        match *self {
            BallBall(ref cd)         => cd.num_colls(),
            BallPlane(ref cd)        => cd.num_colls(),
            PlaneBall(ref cd)        => cd.num_colls(),
            BallImplicit(ref cd)     => cd.num_colls(),
            ImplicitBall(ref cd)     => cd.num_colls(),
            PlaneImplicit(ref cd)    => cd.num_colls(),
            ImplicitPlane(ref cd)    => cd.num_colls(),
            ImplicitImplicit(ref cd) => cd.num_colls(),
            CompoundCompound(ref cd) => cd.num_colls(),
            CompoundAny(ref cd)      => cd.num_colls(),
            AnyCompound(ref cd)      => cd.num_colls(),
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, LV>]) {
        match *self {
            BallBall(ref cd)         => cd.colls(out_colls),
            BallPlane(ref cd)        => cd.colls(out_colls),
            PlaneBall(ref cd)        => cd.colls(out_colls),
            BallImplicit(ref cd)     => cd.colls(out_colls),
            ImplicitBall(ref cd)     => cd.colls(out_colls),
            PlaneImplicit(ref cd)    => cd.colls(out_colls),
            ImplicitPlane(ref cd)    => cd.colls(out_colls),
            ImplicitImplicit(ref cd) => cd.colls(out_colls),
            CompoundCompound(ref cd) => cd.colls(out_colls),
            CompoundAny(ref cd)      => cd.colls(out_colls),
            AnyCompound(ref cd)      => cd.colls(out_colls)
        }
    }

    #[inline]
    fn toi(_:    Option<GeomGeom<N, LV, AV, M>>,
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &Geom<N, LV, M>,
           m2:   &M,
           g2:   &Geom<N, LV, M>)
           -> Option<N> {
        toi(m1, dir, dist, g1, m2, g2)
    }
}

/// Computes the time of impact of two `Geom`.
#[inline]
pub fn toi<N:  Send + Freeze + ApproxEq<N> + Num + Real + Float + Ord + Cast<f32> + Clone + Algebraic,
           LV: 'static + Send + Freeze + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> +
               Translation<LV> + Clone + Rotate<LV> + Transform<LV>,
           AV: Vec<N>,
           M:  Send + Freeze + Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> +
               AbsoluteRotate<LV> + Mul<M, M> + Inv + One>(
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &Geom<N, LV, M>,
           m2:   &M,
           g2:   &Geom<N, LV, M>)
           -> Option<N> {
    match (g1, g2) {
        (&ImplicitGeom(BallGeom(ref b1)), &ImplicitGeom(BallGeom(ref b2))) => toi::ball_ball(m1, dir, b1, m2, b2),
        (&ImplicitGeom(ref i), &PlaneGeom(ref p))      => toi::plane_implicit(m2, p, m1, dir, i),
        (&PlaneGeom(ref p), &ImplicitGeom(ref i))      => toi::plane_implicit(m1, p, m2, &-dir, i),
        (&ImplicitGeom(ref i1), &ImplicitGeom(ref i2)) => toi::implicit_implicit(m1, dir, i1, m2, i2),
        (&CompoundGeom(_), &CompoundGeom(_))   => fail!("Not yet implemented."), // CompoundCompound(),
        (&CompoundGeom(ref c), b) => {
            CollisionDetector::toi(None::<CA<N, LV, AV, M>>, m1, dir, dist, c.get(), m2, b)
        },
        (a, &CompoundGeom(ref c)) => {
            CollisionDetector::toi(None::<AC<N, LV, AV, M>>, m1, dir, dist, a, m2, c.get())
        },
        _ => fail!("Cannot compute the toi of those two geometries.")
    }
}

struct Dispatcher<N, LV, AV, M> {
    simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>
}

impl<N: Clone, LV: Clone, AV, M>
Dispatcher<N, LV, AV, M> {
    fn new(simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>) -> Dispatcher<N, LV, AV, M> {
        Dispatcher {
            simplex: simplex
        }
    }
}

impl<N: Send + Freeze + Zero + Cast<f32> + Clone, LV: Send + Freeze + Clone, AV, M: Send + Freeze>
     broad::Dispatcher<Geom<N, LV, M>, GeomGeom<N, LV, AV, M>>
for Dispatcher<N, LV, AV, M> {
    fn dispatch(&self, g1: &Geom<N, LV, M>, g2: &Geom<N, LV, M>)
                -> GeomGeom<N, LV, AV, M> {
        GeomGeom::new(g1, g2, &self.simplex)
    }

    fn is_valid(&self, _: &Geom<N, LV, M>, _: &Geom<N, LV, M>) -> bool {
        true
    }
}

impl<N: Clone, LV: Clone, AV, M> Clone for Dispatcher<N, LV, AV, M> {
    fn clone(&self) -> Dispatcher<N, LV, AV, M> {
        Dispatcher::new(self.simplex.clone())
    }
}
