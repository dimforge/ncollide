//! Collision detector between two `Box<Geom>`.

use std::num::Bounded;
use std::intrinsics::TypeId;
use std::any::{Any, AnyRefExt};
use std::collections::HashMap;
use sync::{Arc, RWLock};
use nalgebra::na;
use geom::{AnnotatedPoint, Geom, ConcaveGeom, Cone, Cuboid, Ball, Capsule, Convex, Cylinder,
           Compound, Mesh, Triangle, Segment, Plane};
use implicit::{Implicit, PreferedSamplingDirections};
use narrow::algorithm::simplex::Simplex;
use narrow::algorithm::johnson_simplex::{JohnsonSimplex, RecursionTemplate};
use narrow::{CollisionDetector, ImplicitImplicit, BallBall,
             ImplicitPlane, PlaneImplicit, ConcaveGeomGeomFactory, GeomConcaveGeomFactory,
             BezierSurfaceBall, BallBezierSurface, BezierSurfaceBezierSurface,
             Contact};
use narrow::surface_selector::HyperPlaneSurfaceSelector;
use narrow::surface_subdivision_tree::SurfaceSubdivisionTreeCache;
use narrow::OneShotContactManifoldGenerator as OSCMG;
use math::{Scalar, Vect, Matrix};

/// Same as the `CollisionDetector` trait but using dynamic dispatch on the geometries.
pub trait GeomGeomCollisionDetector {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self,
              &GeomGeomDispatcher,
              &Matrix,
              &Geom,
              &Matrix,
              &Geom);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact>);
}

/// Trait to be implemented by collision detector using dynamic dispatch.
///
/// This is used to know the exact type of the geometries.
pub trait DynamicCollisionDetector<G1, G2>: GeomGeomCollisionDetector { }

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

impl<D: CollisionDetector<G1, G2>, G1, G2>
CollisionDetector<G1, G2> for DetectorWithoutRedispatch<D> {
    fn update(&mut self, _: &Matrix, _: &G1, _: &Matrix, _: &G2) { unreachable!() }
    fn num_colls(&self) -> uint { unreachable!() }
    fn colls(&self, _: &mut Vec<Contact>) { unreachable!() }
    fn toi(_: Option<DetectorWithoutRedispatch<D>>, _: &Matrix, _: &Vect, _: &Scalar, _: &G1, _: &Matrix, _: &G2) -> Option<Scalar> {
        unreachable!()
    }
}

impl<D: CollisionDetector<G1, G2>, G1, G2>
DynamicCollisionDetector<G1, G2> for DetectorWithoutRedispatch<D> { }

impl<D: CollisionDetector<G1, G2>, G1: 'static, G2: 'static>
GeomGeomCollisionDetector for DetectorWithoutRedispatch<D> {
    #[inline]
    fn update(&mut self,
              _:  &GeomGeomDispatcher,
              m1: &Matrix,
              g1: &Geom,
              m2: &Matrix,
              g2: &Geom) {
        self.detector.update(
            m1,
            g1.downcast_ref::<G1>().expect("Invalid geometry."),
            m2,
            g2.downcast_ref::<G2>().expect("Invalid geometry."))
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.detector.num_colls()
    }

    #[inline]
    fn colls(&self, cs: &mut Vec<Contact>) {
        self.detector.colls(cs)
    }
}

/// Collision dispatcher between two `~Geom`.
pub struct GeomGeomDispatcher {
    constructors: HashMap<(TypeId, TypeId), Box<CollisionDetectorFactory>>
}

impl GeomGeomDispatcher {
    /// Creates a new `GeomGeomDispatcher` without the default set of collision detectors
    /// factories.
    pub fn new_without_default() -> GeomGeomDispatcher {
        GeomGeomDispatcher {
            constructors: HashMap::new()
        }
    }

    /// Registers a new collision detection algorithm factory for a pair of geometries.
    ///
    /// This is unsafe because there is no way to check that the factory will really generate
    /// collision detectors suited for `G1` and `G2`. Whenever possible, use `register_detector` or
    /// `register_dynamic_detector` instead.
    pub unsafe fn register_factory<G1: 'static + Any,
                                   G2: 'static + Any,
                                   F:  CollisionDetectorFactory>(
                                   &mut self,
                                   factory: F) {
        let key = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.insert(key, box factory as Box<CollisionDetectorFactory>);
    }

    /// Registers a new dynamic collision detector for two geometries.
    pub fn register_dynamic_detector<G1: 'static + Any,
                                     G2: 'static + Any,
                                     D:  'static + Send + Clone +
                                         DynamicCollisionDetector<G1, G2>>(
                                     &mut self,
                                     d:   D) {
        let factory = CollisionDetectorCloner::new(d);
        unsafe { self.register_factory::<G1, G2, CollisionDetectorCloner<D>>(factory) }
    }

    /// Registers a new collision detector for two geometries.
    pub fn register_detector<G1: 'static + Any,
                             G2: 'static + Any,
                             D:  'static + Send + CollisionDetector<G1, G2> + Clone>(
                             &mut self,
                             d:   D) {
        self.register_dynamic_detector(DetectorWithoutRedispatch::new(d));
    }

    /// Unregister the collision detector for a givem pair of geometries.
    pub fn unregister_detector<G1: 'static + Any, G2: 'static + Any>(&mut self) {
        let key = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.remove(&key);
    }

    /// If registered, creates a new collision detector adapted for the two given geometries.
    pub fn dispatch(&self, a: &Geom, b: &Geom) -> Option<Box<GeomGeomCollisionDetector>> {
        self.constructors.find(&(a.get_dyn_type_id(), b.get_dyn_type_id())).map(|f| f.build())
    }
}

impl GeomGeomDispatcher {
    // FIXME: make this a function which has the simplex and the prediction margin as parameters
    /// Creates a new `GeomGeomDispatcher` able do build collision detectors for any valid pair of
    /// geometries supported by `ncollide`.
    pub fn new() -> GeomGeomDispatcher {
        let mut res = GeomGeomDispatcher::new_without_default();

        type Simplex  = JohnsonSimplex<AnnotatedPoint>;
        type Self     = GeomGeomDispatcher;
        type Super    = Box<GeomGeomCollisionDetector>;

        /*
         * Involving a Plane
         */
        let prediction: &Scalar = &na::cast(0.1f64);

        // Ball vs. Ball
        let bb = BallBall::new(prediction.clone());
        res.register_detector(bb);

        // Ball vs Bézier
        let selector = HyperPlaneSurfaceSelector::new(Bounded::max_value());
        let cache    = Arc::new(RWLock::new(SurfaceSubdivisionTreeCache::new()));
        let bs       = BallBezierSurface::new(selector.clone(), prediction.clone(), cache.clone());
        let sb       = BezierSurfaceBall::new(selector.clone(), prediction.clone(), cache.clone());
        res.register_detector(bs);
        res.register_detector(sb);

        // Bézier vs. Bézier
        let ss = BezierSurfaceBezierSurface::new(selector.clone(), prediction.clone(), cache.clone());
        res.register_detector(ss);

        // Plane vs. Implicit
        res.register_default_plane_implicit_detector::<Ball>(false, prediction);
        res.register_default_plane_implicit_detector::<Cuboid >(true, prediction);
        res.register_default_plane_implicit_detector::<Cone>(true, prediction);
        res.register_default_plane_implicit_detector::<Cylinder>(true, prediction);
        res.register_default_plane_implicit_detector::<Capsule>(true, prediction);
        res.register_default_plane_implicit_detector::<Convex>(true, prediction);
        res.register_default_plane_implicit_detector::<Triangle>(true, prediction);
        res.register_default_plane_implicit_detector::<Segment>(true, prediction);

        // Implicit vs. Implicit
        // NOTE: some pair will be registered twice…
        res.register_default_implicit_detectors::<Cuboid>(true, prediction);
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
        res.register_default_concave_geom_geom_detector::<Compound, Cuboid>();
        res.register_default_concave_geom_geom_detector::<Compound, Cone>();
        res.register_default_concave_geom_geom_detector::<Compound, Cylinder>();
        res.register_default_concave_geom_geom_detector::<Compound, Capsule>();
        res.register_default_concave_geom_geom_detector::<Compound, Convex>();
        res.register_default_concave_geom_geom_detector::<Compound, Triangle>();
        res.register_default_concave_geom_geom_detector::<Compound, Segment>();

        // TriangleMesh vs. Other
        res.register_default_concave_geom_geom_detector::<Mesh, Plane>();
        res.register_default_concave_geom_geom_detector::<Mesh, Ball>();
        res.register_default_concave_geom_geom_detector::<Mesh, Cuboid>();
        res.register_default_concave_geom_geom_detector::<Mesh, Cone>();
        res.register_default_concave_geom_geom_detector::<Mesh, Cylinder>();
        res.register_default_concave_geom_geom_detector::<Mesh, Capsule>();
        res.register_default_concave_geom_geom_detector::<Mesh, Convex>();
        res.register_default_concave_geom_geom_detector::<Mesh, Triangle>();
        res.register_default_concave_geom_geom_detector::<Mesh, Segment>();

        // FIXME: implement a ConcaveGeomConcaveGeom detector?
        res.register_default_concave_geom_geom_detector::<Compound, Compound>();
        res.register_default_concave_geom_geom_detector::<Mesh, Compound>();

        res
    }

    /// Registers a `PlaneImplicit` collision detector between a given implicit geometry and a plane.
    pub fn register_default_plane_implicit_detector<I: 'static + Implicit<Vect, Matrix>>(
                                                    &mut self,
                                                    generate_manifold: bool,
                                                    prediction:        &Scalar) {
        let p = if generate_manifold { na::zero() } else { prediction.clone() };

        let d1 = ImplicitPlane::<I>::new(p.clone());
        let d2 = PlaneImplicit::<I>::new(p.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    /// Register an `ImplicitImplicit` collision detector between two implicit geometries.
    pub fn register_default_implicit_implicit_detector<G1: 'static            +
                                                           Implicit<Vect, Matrix> +
                                                           PreferedSamplingDirections<Vect, Matrix>,
                                                       G2: 'static            +
                                                           Implicit<Vect, Matrix> +
                                                           PreferedSamplingDirections<Vect, Matrix>,
                                                       S:  Send + Clone + Simplex<AnnotatedPoint>>(
                                                       &mut self,
                                                       generate_manifold: bool,
                                                       prediction:        &Scalar,
                                                       simplex:           &S) {
        let p = if generate_manifold { na::zero() } else { prediction.clone() };

        let d1 = ImplicitImplicit::<S, G1, G2>::new(p.clone(), simplex.clone());
        let d2 = ImplicitImplicit::<S, G2, G1>::new(p.clone(), simplex.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    /// Register an `ConcaveGeomGeom` collision detector between a given concave geometry and a
    /// given geometry.
    pub fn register_default_concave_geom_geom_detector<G1: 'static + ConcaveGeom,
                                                       G2: 'static + Geom>(&mut self) {
        let  f1 = ConcaveGeomGeomFactory::<G1, G2>;
        let  f2 = GeomConcaveGeomFactory::<G2, G1>;

        // FIXME: find a way to factorize that?
        unsafe { self.register_factory::<G1, G2, ConcaveGeomGeomFactory<G1, G2>>(f1) }
        unsafe { self.register_factory::<G2, G1, GeomConcaveGeomFactory<G2, G1>>(f2) }
    }

    /// Register a given collision detector and adds it a contact manifold generator (a
    /// `OneShotContactManifoldGenerator`).
    pub fn register_detector_with_contact_manifold_generator<G1: 'static + Any,
                                                             G2: 'static + Any,
                                                             D:  'static + Send +
                                                                 CollisionDetector<G1, G2> +
                                                                 Clone>(
                                                             &mut self,
                                                             d:          D,
                                                             prediction: &Scalar) {
        let d = OSCMG::<D>::new(prediction.clone(), d);
        self.register_detector(d);
    }

    /*
    pub fn register_default_compound_any_detector<G, S>(&mut self) {
        type CA = CompoundAABBAny<~Geom>;
    }
    */

    /// Register `ImplicitImplicit` collision detectors between a given geometry and every implicit
    /// geometry supported by `ncollide`.
    pub fn register_default_implicit_detectors<G: 'static + Implicit<Vect, Matrix> + PreferedSamplingDirections<Vect, Matrix>>(
                                               &mut self,
                                               generate_manifold: bool,
                                               prediction:        &Scalar) {
        type Simplex  = JohnsonSimplex<AnnotatedPoint>;

        // Implicit vs. Implicit
        let rt = RecursionTemplate::new(na::dim::<Vect>());
        let js = &JohnsonSimplex::new(rt);

        self.register_default_implicit_implicit_detector::<Ball, G, Simplex>(false, prediction, js);
        self.register_default_implicit_implicit_detector::<Cuboid, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cone, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cylinder, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Capsule, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Convex, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Triangle, G, Simplex>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Segment, G, Simplex>(generate_manifold, prediction, js);
    }
}

// FIXME: rename that GeomGeomCollisionDetectorFactory ?
/// Trait of structures able do build a new collision detector.
pub trait CollisionDetectorFactory : Send {
    /// Builds a new collision detector.
    fn build(&self) -> Box<GeomGeomCollisionDetector>;
}

/// Cloning-based collision detector factory.
pub struct CollisionDetectorCloner<CD> {
    template: CD
}

impl<CD: GeomGeomCollisionDetector + Clone> CollisionDetectorCloner<CD> {
    /// Creates a new `CollisionDetectorCloner`.
    ///
    /// The cloned detector is `CD`.
    fn new(detector: CD) -> CollisionDetectorCloner<CD> {
        CollisionDetectorCloner {
            template: detector
        }
    }
}

impl<CD: 'static + Send + GeomGeomCollisionDetector + Clone>
CollisionDetectorFactory for CollisionDetectorCloner<CD> {
    fn build(&self) -> Box<GeomGeomCollisionDetector> {
        box self.template.clone() as Box<GeomGeomCollisionDetector>
    }
}
