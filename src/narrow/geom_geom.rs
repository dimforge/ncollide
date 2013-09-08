use std::num::{Zero, One};
use extra::serialize::{Decodable, Decoder};
use nalgebra::mat::{Translation, Rotate, Rotation, Transform, Inv};
use nalgebra::vec::{Vec, AlgebraicVecExt, Cross};
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
type C<N, LV, M, II> = CompoundAABB<N, LV, M, Geom<N, LV, M, II>>;
type CA<N, LV, AV, M, II> = CompoundAABBAny<N, LV, M,
                                            Geom<N, LV, M, II>,
                                            Dispatcher<N, LV, AV, M, II>,
                                            GeomGeom<N, LV, AV, M, II>>;
type AC<N, LV, AV, M, II> = AnyCompoundAABB<N, LV, M,
                                            Geom<N, LV, M, II>,
                                            Dispatcher<N, LV, AV, M, II>,
                                            GeomGeom<N, LV, AV, M, II>>;

#[deriving(Encodable)]
pub enum GeomGeom<N, LV, AV, M, II> {
    BallBall(BallBall<N, LV, M>),
    BallPlane(ImplicitPlane<N, LV, M, Ball<N>>),
    PlaneBall(PlaneImplicit<N, LV, M, Ball<N>>),
    BallImplicit(ImplicitImplicit<S<N, LV>, Ball<N>, IGeom<N, LV, M>, N, LV>),
    ImplicitBall(ImplicitImplicit<S<N, LV>, IGeom<N, LV, M>, Ball<N>, N, LV>),
    PlaneImplicit(OSCMG<PlaneImplicit<N, LV, M, IGeom<N, LV, M>>, N, LV, AV, M>),
    ImplicitPlane(OSCMG<ImplicitPlane<N, LV, M, IGeom<N, LV, M>>, N, LV, AV, M>),
    ImplicitImplicit(OSCMG<ImplicitImplicit<S<N, LV>, IGeom<N, LV, M>, IGeom<N, LV, M>, N, LV>, N, LV, AV, M>),
    CompoundCompound(CompoundAABBCompoundAABB<N, LV, M,
                                              Geom<N, LV, M, II>,
                                              Dispatcher<N, LV, AV, M, II>,
                                              GeomGeom<N, LV, AV, M, II>>),
    CompoundAny(CA<N, LV, AV, M, II>),
    AnyCompound(AC<N, LV, AV, M, II>)
}

impl<N: NumCast + Zero + Clone, LV: Clone, AV, M, II> GeomGeom<N, LV, AV, M, II> {
    /// Creates a new Geom vs Geom collision detector.
    pub fn new(g1:     &Geom<N, LV, M, II>,
               g2:     &Geom<N, LV, M, II>,
               s:      &JohnsonSimplex<N, AnnotatedPoint<LV>>)
               -> GeomGeom<N, LV, AV, M, II> {
        match (g1, g2) {
            (&ImplicitGeom(BallGeom(_)), &ImplicitGeom(BallGeom(_))) => {
                BallBall(BallBall::new(NumCast::from(0.1)))
            },
            (&ImplicitGeom(BallGeom(_)), &PlaneGeom(_))           => {
                BallPlane(ImplicitPlane::new(NumCast::from(0.1)))
            },
            (&PlaneGeom(_), &ImplicitGeom(BallGeom(_)))           => {
                PlaneBall(PlaneImplicit::new(NumCast::from(0.1)))
            },
            (&ImplicitGeom(_), &ImplicitGeom(BallGeom(_)))        => {
                ImplicitBall(ImplicitImplicit::new(NumCast::from(0.1), s.clone()))
            },
            (&ImplicitGeom(BallGeom(_)), &ImplicitGeom(_))        => {
                BallImplicit(ImplicitImplicit::new(NumCast::from(0.1), s.clone()))
            },
            (&ImplicitGeom(_), &PlaneGeom(_))       => {
                ImplicitPlane(OSCMG::new(NumCast::from(0.1), ImplicitPlane::new(Zero::zero())))
            },
            (&PlaneGeom(_), &ImplicitGeom(_))       => {
                PlaneImplicit(OSCMG::new(NumCast::from(0.1), PlaneImplicit::new(Zero::zero())))
            },
            (&ImplicitGeom(_), &ImplicitGeom(_))    => {
                ImplicitImplicit(
                OSCMG::new(NumCast::from(0.1), ImplicitImplicit::new(Zero::zero(), s.clone())))
            },
            (&CompoundGeom(c1), &CompoundGeom(c2))  => {
                CompoundCompound(CompoundAABBCompoundAABB::new(Dispatcher::new(s.clone()), c1, c2))
            },
            (&CompoundGeom(c), _)               => {
                CompoundAny(CompoundAABBAny::new(Dispatcher::new(s.clone()), c))
            },
            (_, &CompoundGeom(c))               => {
                AnyCompound(AnyCompoundAABB::new(Dispatcher::new(s.clone()), c))
            },
            _ => fail!("Dont know how to dispatch that.")
        }
    }
}

/**
 * Collision detector between two `Geometry`. Note that this is only a
 * wrapper on the collision detector specific to each geometry.
 */
impl<N: ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
     LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone + ToStr +
         Rotate<LV> + Transform<LV>,
     AV: Vec<N> + ToStr,
     M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One,
     II>
CollisionDetector<N, LV, M, Geom<N, LV, M, II>, Geom<N, LV, M, II>>
for GeomGeom<N, LV, AV, M, II> {
    #[inline]
    fn update(&mut self,
              m1: &M,
              g1: &Geom<N, LV, M, II>,
              m2: &M,
              g2: &Geom<N, LV, M, II>) {
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
    fn toi(_:    Option<GeomGeom<N, LV, AV, M, II>>,
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &Geom<N, LV, M, II>,
           m2:   &M,
           g2:   &Geom<N, LV, M, II>)
           -> Option<N> {
        toi(m1, dir, dist, g1, m2, g2)
    }
}

/// Computes the time of impact of two `Geom`.
#[inline]
pub fn toi<N:  ApproxEq<N> + Num + Real + Float + Ord + Clone + ToStr + Algebraic,
           LV: 'static + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> + Clone +
               Rotate<LV> + Transform<LV> + ToStr,
           AV: Vec<N> + ToStr,
           M:  Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> + Mul<M, M> + Inv + One,
           II>(
           m1:   &M,
           dir:  &LV,
           dist: &N,
           g1:   &Geom<N, LV, M, II>,
           m2:   &M,
           g2:   &Geom<N, LV, M, II>)
           -> Option<N> {
    match (g1, g2) {
        (&ImplicitGeom(BallGeom(ref b1)), &ImplicitGeom(BallGeom(ref b2))) => toi::ball_ball(m1, dir, b1, m2, b2),
        (&ImplicitGeom(ref i), &PlaneGeom(ref p))      => toi::plane_implicit(m2, p, m1, dir, i),
        (&PlaneGeom(ref p), &ImplicitGeom(ref i))      => toi::plane_implicit(m1, p, m2, &-dir, i),
        (&ImplicitGeom(ref i1), &ImplicitGeom(ref i2)) => toi::implicit_implicit(m1, dir, i1, m2, i2),
        (&CompoundGeom(_), &CompoundGeom(_))   => fail!("Not yet implemented."), // CompoundCompound(),
        (&CompoundGeom(c), b) => {
            CollisionDetector::toi(None::<CA<N, LV, AV, M, II>>, m1, dir, dist, c, m2, b)
        },
        (a, &CompoundGeom(c)) => {
            CollisionDetector::toi(None::<AC<N, LV, AV, M, II>>, m1, dir, dist, a, m2, c)
        },
        _ => fail!("Cannot compute the toi of those two geometries.")
    }
}

#[deriving(Encodable, Decodable)]
struct Dispatcher<N, LV, AV, M, II> {
    simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>
}

impl<N: Clone, LV: Clone, AV, M, II>
Dispatcher<N, LV, AV, M, II> {
    fn new(simplex: JohnsonSimplex<N, AnnotatedPoint<LV>>) -> Dispatcher<N, LV, AV, M, II> {
        Dispatcher {
            simplex: simplex
        }
    }
}

impl<N: NumCast + Zero + Clone, LV: Clone, AV, M, II>
     broad::Dispatcher<Geom<N, LV, M, II>, GeomGeom<N, LV, AV, M, II>>
for Dispatcher<N, LV, AV, M, II> {
    fn dispatch(&self, g1: &Geom<N, LV, M, II>, g2: &Geom<N, LV, M, II>)
                -> GeomGeom<N, LV, AV, M, II> {
        GeomGeom::new(g1, g2, &self.simplex)
    }

    fn is_valid(&self, _: &Geom<N, LV, M, II>, _: &Geom<N, LV, M, II>) -> bool {
        true
    }
}

impl<N: Clone, LV: Clone, AV, M, II> Clone for Dispatcher<N, LV, AV, M, II> {
    fn clone(&self) -> Dispatcher<N, LV, AV, M, II> {
        Dispatcher::new(self.simplex.clone())
    }
}

impl<N:  'static + Decodable<D> + Algebraic + Primitive + Orderable + Signed + Clone + ToStr,
     LV: 'static + Decodable<D> + AlgebraicVecExt<N> + Clone + ToStr,
     AV: Decodable<D>,
     M:  Decodable<D> + Translation<LV> + Rotate<LV> + Transform<LV> + Mul<M, M>,
     II: Decodable<D>,
     D: Decoder>
Decodable<D> for GeomGeom<N, LV, AV, M, II> {
    fn decode(d: &mut D) -> GeomGeom<N, LV, AV, M, II> {

        let variants = [
            "BallBall"
            , "BallPlane"
            , "PlaneBall"
            , "BallImplicit"
            , "ImplicitBall"
            , "PlaneImplicit"
            , "ImplicitPlane"
            , "ImplicitImplicit"
            , "CompoundCompound"
            , "CompoundAny"
            , "AnyCompound"
        ];

        // FIXME: write a macro for that?
        do d.read_enum_variant(variants) |d, i| {
            match i {
                0 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        BallBall(Decodable::decode(d))
                    }
                },
                1 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        BallPlane(Decodable::decode(d))
                    }
                },
                2 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        BallImplicit(Decodable::decode(d))
                    }
                },
                3 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        ImplicitBall(Decodable::decode(d))
                    }
                },
                4 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        PlaneImplicit(Decodable::decode(d))
                    }
                },
                5 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        ImplicitPlane(Decodable::decode(d))
                    }
                },
                6 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        ImplicitImplicit(Decodable::decode(d))
                    }
                },
                7 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        CompoundCompound(Decodable::decode(d))
                    }
                },
                8 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        CompoundAny(Decodable::decode(d))
                    }
                },
                9 => {
                    do d.read_enum_variant_arg(0u) |d| {
                        AnyCompound(Decodable::decode(d))
                    }
                },
                _ => fail!("Trying to decode an unknown variant.")
            }
        }
    }
}
