use std::num::{One, Zero};
use std::unstable::intrinsics::TypeId;
use std::hashmap::HashMap;
use nalgebra::na;
use nalgebra::na::{Cast, Translation, Rotate, Rotation, AbsoluteRotate, Transform,
                   Inv, Vec, AlgebraicVecExt, Cross, Indexable};
use volumetric::InertiaTensor;
use geom::{AnnotatedPoint, Geom, ConcaveGeom};
use geom;
use implicit::Implicit;
use contact::Contact;
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::{JohnsonSimplex, RecursionTemplate};
use narrow::algorithm::minkowski_sampling::PreferedSamplingDirections;
use narrow::{CollisionDetector, ImplicitImplicit, BallBall,
             ImplicitPlane, PlaneImplicit, ConcaveGeomGeomFactory, GeomConcaveGeomFactory};
use OSCMG = narrow::OneShotContactManifoldGenerator;

/// Same as the `CollisionDetector` trait but using dynamic dispatch on the geometries.
pub trait GeomGeomCollisionDetector<N, LV, AV, M, II> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self,
              &GeomGeomDispatcher<N, LV, AV, M, II>,
              &M,
              &Geom<N, LV, M, II>,
              &M,
              &Geom<N, LV, M, II>);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut ~[Contact<N, LV>]);
}

/// Trait to be implemented by collision detector using dynamic dispatch.
/// This is used to know the exact type of the geometries.
pub trait DynamicCollisionDetector<N, LV, AV, M, II, G1, G2>: GeomGeomCollisionDetector<N, LV, AV, M, II> { }

#[deriving(Clone)]
struct DetectorWithoutRedispatch<D> {
    detector: D
}

impl<D> DetectorWithoutRedispatch<D> {
    pub fn new(d: D) -> DetectorWithoutRedispatch<D> {
        DetectorWithoutRedispatch {
            detector: d
        }
    }
}

impl<D: CollisionDetector<N, V, M, G1, G2>, N, V, M, G1, G2>
CollisionDetector<N, V, M, G1, G2> for DetectorWithoutRedispatch<D> {
    fn update(&mut self, _: &M, _: &G1, _: &M, _: &G2) { unreachable!() }
    fn num_colls(&self) -> uint { unreachable!() }
    fn colls(&self, _: &mut ~[Contact<N, V>]) { unreachable!() }
    fn toi(_: Option<DetectorWithoutRedispatch<D>>, _: &M, _: &V, _: &N, _: &G1, _: &M, _: &G2) -> Option<N> {
        unreachable!()
    }
}

impl<D: CollisionDetector<N, LV, M, G1, G2>, N, LV, AV, M, II, G1, G2>
DynamicCollisionDetector<N, LV, AV, M, II, G1, G2> for DetectorWithoutRedispatch<D> { }

impl<D: CollisionDetector<N, LV, M, G1, G2>, N, LV, AV, M, II, G1: 'static, G2: 'static>
GeomGeomCollisionDetector<N, LV, AV, M, II> for DetectorWithoutRedispatch<D> {
    #[inline]
    fn update(&mut self,
              _:  &GeomGeomDispatcher<N, LV, AV, M, II>,
              m1: &M,
              g1: &Geom<N, LV, M, II>,
              m2: &M,
              g2: &Geom<N, LV, M, II>) {
        self.detector.update(
            m1,
            g1.as_ref::<G1>().expect("Invalid geometry."),
            m2,
            g2.as_ref::<G2>().expect("Invalid geometry."))
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.detector.num_colls()
    }

    #[inline]
    fn colls(&self, cs: &mut ~[Contact<N, LV>]) {
        self.detector.colls(cs)
    }
}

pub struct GeomGeomDispatcher<N, LV, AV, M, II> {
    priv constructors: HashMap<(TypeId, TypeId), ~CollisionDetectorFactory<N, LV, AV, M, II>>
}

impl<N, LV, AV, M, II> GeomGeomDispatcher<N, LV, AV, M, II> {
    pub fn new_without_default() -> GeomGeomDispatcher<N, LV, AV, M, II> {
        GeomGeomDispatcher {
            constructors: HashMap::new()
        }
    }

    pub fn register_dynamic_detector<G1: 'static + Any,
                                     G2: 'static + Any,
                                     D:  'static + Send + Clone +
                                         DynamicCollisionDetector<N, LV, AV, M, II, G1, G2>>(
                                     &mut self,
                                     d:   D) {
        let key     = (TypeId::of::<G1>(), TypeId::of::<G2>());
        let factory = ~CollisionDetectorCloner::new(d);
        self.constructors.insert(key, factory as ~CollisionDetectorFactory<N, LV, AV, M, II>);
    }

    pub fn register_detector<G1: 'static + Any,
                             G2: 'static + Any,
                             D:  'static + Send + CollisionDetector<N, LV, M, G1, G2> + Clone>(
                             &mut self,
                             d:   D) {
        self.register_dynamic_detector(DetectorWithoutRedispatch::new(d));
    }

    pub fn unregister_detector<G1: 'static + Any, G2: 'static + Any>(&mut self) {
        let key = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.remove(&key);
    }

    pub fn dispatch(&self, a: &Geom<N, LV, M, II>, b: &Geom<N, LV, M, II>)
                    -> ~GeomGeomCollisionDetector<N, LV, AV, M, II> {
        match self.constructors.find(&(a.get_type_id(), b.get_type_id())) {
            Some(f) => f.build(),
            None    => fail!("Unable to find a collision detector.")
        }
    }
}

impl<N:  'static + Send + Freeze + ApproxEq<N> + Num + Real + Float + Ord + Cast<f32> + Clone + Algebraic,
     LV: 'static + Send + Freeze + AlgebraicVecExt<N> + Cross<AV> + ApproxEq<N> + Translation<LV> +
         Clone + Rotate<LV> + Transform<LV>,
     AV: 'static + Clone + Vec<N>,
     M:  'static + Clone + Send + Freeze + Rotation<AV> + Rotate<LV> + Translation<LV> + Transform<LV> +
        AbsoluteRotate<LV> + Mul<M, M> + Inv + One,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M> + Indexable<(uint, uint), N>>
GeomGeomDispatcher<N, LV, AV, M, II> {
    // FIXME: make this a function which has the simplex and the prediction margin as parameters
    pub fn new() -> GeomGeomDispatcher<N, LV, AV, M, II> {
        let mut res = GeomGeomDispatcher::new_without_default();

        type Simplex  = JohnsonSimplex<N, AnnotatedPoint<LV>>;
        type Self     = GeomGeomDispatcher<N, LV, AV, M, II>;
        type Super    = ~GeomGeomCollisionDetector<N, LV, AV, M, II>;

        type Ball     = geom::Ball<N>;
        type Box      = geom::Box<N, LV>;
        type Plane    = geom::Plane<N, LV>;
        type Cone     = geom::Cone<N>;
        type Cylinder = geom::Cylinder<N>;
        type Capsule  = geom::Capsule<N>;
        type Convex   = geom::Convex<N, LV>;
        type Compound = geom::Compound<N, LV, M, II>;
        type Triangle = geom::Triangle<N, LV>;
        type Segment  = geom::Segment<N, LV>;
        type TriangleMesh = geom::Mesh<N, LV, M, II, Triangle>;
        type SegmentStrip = geom::Mesh<N, LV, M, II, Segment>;

        /*
         * Involving a Plane
         */
        let prediction: &N = &na::cast(0.1);

        // Ball vs. Ball
        let bb: BallBall<N, LV, M> = BallBall::new(prediction.clone());
        res.register_detector(bb);

        // Plane vs. Implicit
        res.register_default_plane_implicit_detector::<Ball>(false, prediction);
        res.register_default_plane_implicit_detector::<Box>(true, prediction);
        res.register_default_plane_implicit_detector::<Cone>(true, prediction);
        res.register_default_plane_implicit_detector::<Cylinder>(true, prediction);
        res.register_default_plane_implicit_detector::<Capsule>(true, prediction);
        res.register_default_plane_implicit_detector::<Convex>(true, prediction);
        res.register_default_plane_implicit_detector::<Triangle>(true, prediction);
        res.register_default_plane_implicit_detector::<Segment>(true, prediction);

        // Implicit vs. Implicit
        // NOTE: some pair will be registered twice…
        res.register_default_implicit_detectors::<Box>(true, prediction);
        res.register_default_implicit_detectors::<Cone>(true, prediction);
        res.register_default_implicit_detectors::<Cylinder>(true, prediction);
        res.register_default_implicit_detectors::<Capsule>(true, prediction);
        res.register_default_implicit_detectors::<Convex>(true, prediction);
        res.register_default_implicit_detectors::<Triangle>(true, prediction);
        res.register_default_implicit_detectors::<Segment>(true, prediction);

        // FIXME: refactor the three following blocks?
        // Compound vs. Other
        res.register_default_concave_geom_geom_detector::<Compound, Plane>();
        res.register_default_concave_geom_geom_detector::<Compound, Ball>();
        res.register_default_concave_geom_geom_detector::<Compound, Box>();
        res.register_default_concave_geom_geom_detector::<Compound, Cone>();
        res.register_default_concave_geom_geom_detector::<Compound, Cylinder>();
        res.register_default_concave_geom_geom_detector::<Compound, Capsule>();
        res.register_default_concave_geom_geom_detector::<Compound, Convex>();
        res.register_default_concave_geom_geom_detector::<Compound, Triangle>();
        res.register_default_concave_geom_geom_detector::<Compound, Segment>();

        // TriangleMesh vs. Other
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Plane>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Ball>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Box>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Cone>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Cylinder>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Capsule>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Convex>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Triangle>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Segment>();

        // SegmentStrip vs. Other
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Plane>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Ball>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Box>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Cone>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Cylinder>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Capsule>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Convex>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Triangle>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Segment>();

        // FIXME: implement a ConcaveGeomConcaveGeom detector?
        res.register_default_concave_geom_geom_detector::<Compound, Compound>();
        res.register_default_concave_geom_geom_detector::<TriangleMesh, Compound>();
        res.register_default_concave_geom_geom_detector::<SegmentStrip, Compound>();

        res
    }

    pub fn register_default_plane_implicit_detector<I: 'static + Implicit<N, LV, M>>(
                                                    &mut self,
                                                    generate_manifold: bool,
                                                    prediction:        &N) {
        let p = if generate_manifold { na::zero() } else { prediction.clone() };

        type IP    = ImplicitPlane<N, LV, M, I>;
        let d1: IP = ImplicitPlane::new(p.clone());

        type PI    = PlaneImplicit<N, LV, M, I>;
        let d2: PI = PlaneImplicit::new(p.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    pub fn register_default_implicit_implicit_detector<G1: 'static            +
                                                           Implicit<N, LV, M> +
                                                           PreferedSamplingDirections<LV, M>,
                                                       G2: 'static            +
                                                           Implicit<N, LV, M> +
                                                           PreferedSamplingDirections<LV, M>,
                                                       S:  Send + Clone + Simplex<N, AnnotatedPoint<LV>>>(
                                                       &mut self,
                                                       generate_manifold: bool,
                                                       prediction:        &N,
                                                       simplex:           &S) {
        let p = if generate_manifold { na::zero() } else { prediction.clone() };

        type I1I2    = ImplicitImplicit<N, LV, S, G1, G2>;
        let d1: I1I2 = ImplicitImplicit::new(p.clone(), simplex.clone());

        type I2I1    = ImplicitImplicit<N, LV, S, G2, G1>;
        let d2: I2I1 = ImplicitImplicit::new(p.clone(), simplex.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    pub fn register_default_concave_geom_geom_detector<G1: 'static + ConcaveGeom<N, LV, M, II>,
                                                       G2: 'static + Geom<N, LV, M, II>>(&mut self) {
        type CGG      = ConcaveGeomGeomFactory<N, LV, AV, M, II, G1, G2>;
        let  f1: CGG  = ConcaveGeomGeomFactory;

        type GCG      = GeomConcaveGeomFactory<N, LV, AV, M, II, G2, G1>;
        let  f2: GCG  = GeomConcaveGeomFactory;

        // FIXME: find a way to factorize that?
        let key     = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.insert(key, ~f1 as ~CollisionDetectorFactory<N, LV, AV, M, II>);

        let key     = (TypeId::of::<G2>(), TypeId::of::<G1>());
        self.constructors.insert(key, ~f2 as ~CollisionDetectorFactory<N, LV, AV, M, II>);
    }

    pub fn register_detector_with_contact_manifold_generator<G1: 'static + Any,
                                                             G2: 'static + Any,
                                                             D:  'static + Send +
                                                                 CollisionDetector<N, LV, M, G1, G2> +
                                                                 Clone>(
                                                             &mut self,
                                                             d:          D,
                                                             prediction: &N) {
        type OSCMGD   = OSCMG<D, N, LV, AV, M>;
        let d: OSCMGD = OSCMG::new(prediction.clone(), d);
        self.register_detector(d);
    }

    /*
    pub fn register_default_compound_any_detector<G, S>(&mut self) {
        type CA = CompoundAABBAny<N, LV, M, ~Geom<N, LV, M, II>>;
    }
    */

    pub fn register_default_implicit_detectors<G: 'static + Implicit<N, LV, M> + PreferedSamplingDirections<LV, M>>(
                                               &mut self,
                                               generate_manifold: bool,
                                               prediction:        &N) {
        type Simplex  = JohnsonSimplex<N, AnnotatedPoint<LV>>;
        type Ball     = geom::Ball<N>;
        type Box      = geom::Box<N, LV>;
        type Cone     = geom::Cone<N>;
        type Cylinder = geom::Cylinder<N>;
        type Capsule  = geom::Capsule<N>;
        type Convex   = geom::Convex<N, LV>;
        type Triangle = geom::Triangle<N, LV>;
        type Segment  = geom::Segment<N, LV>;

        // Implicit vs. Implicit
        let rt = RecursionTemplate::new(na::dim::<LV>());
        let js = &JohnsonSimplex::new(rt);

        self.register_default_implicit_implicit_detector::<Ball, G, Simplex>(false, prediction, js);
        self.register_default_implicit_implicit_detector::<Box, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cone, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cylinder, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Capsule, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Convex, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Triangle, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Segment, G, Simplex>(generate_manifold, prediction, js);
    }
}

// FIXME: rename that GeomGeomCollisionDetectorFactory ?
pub trait CollisionDetectorFactory<N, LV, AV, M, II> : Send + Freeze {
    fn build(&self) -> ~GeomGeomCollisionDetector<N, LV, AV, M, II>;
}

pub struct CollisionDetectorCloner<CD> {
    template: CD
}

impl<CD: GeomGeomCollisionDetector<N, LV, AV, M, II> + Clone, N, LV, AV, M, II> CollisionDetectorCloner<CD> {
    fn new(detector: CD) -> CollisionDetectorCloner<CD> {
        CollisionDetectorCloner {
            template: detector
        }
    }
}

impl<CD: 'static + Send + Freeze + GeomGeomCollisionDetector<N, LV, AV, M, II> + Clone, N, LV, AV, M, II>
CollisionDetectorFactory<N, LV, AV, M, II> for CollisionDetectorCloner<CD> {
    fn build(&self) -> ~GeomGeomCollisionDetector<N, LV, AV, M, II> {
        ~self.template.clone() as ~GeomGeomCollisionDetector<N, LV, AV, M, II>
    }
}
