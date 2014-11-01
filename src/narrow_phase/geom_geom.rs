//! Collision detector between two `Box<Shape>`.

use std::num::Bounded;
use std::intrinsics::TypeId;
use std::any::{Any, AnyRefExt};
use std::collections::HashMap;
use na::{Translate, Rotation, Cross};
use na;
use shape::{AnnotatedPoint, Shape, ConcaveShape, Cuboid, Convex,
                    Compound, Mesh, Triangle, Segment, Plane, Cone, Cylinder, Ball, Capsule};
use support_map::{SupportMap, PreferedSamplingDirections};
use narrow_phase::algorithm::simplex::Simplex;
use narrow_phase::algorithm::johnson_simplex::{JohnsonSimplex, RecursionTemplate};
use narrow_phase::{CollisionDetector, SupportMapSupportMap, BallBall,
                      SupportMapPlane, PlaneSupportMap, ConcaveShapeShapeFactory, ShapeConcaveShapeFactory,
                      BezierSurfaceBall, BallBezierSurface, Contact};
use narrow_phase::surface_selector::HyperPlaneSurfaceSelector;
use narrow_phase::OneShotContactManifoldGenerator as OSCMG;
use math::{Scalar, Point, Vect, Isometry};


/// Same as the `CollisionDetector` trait but using dynamic dispatch on the geometries.
pub trait ShapeShapeCollisionDetector<N, P, V, M, I> {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn update(&mut self, &ShapeShapeDispatcher<N, P, V, M, I>, &M, &Shape<N, P, V, M>, &M, &Shape<N, P, V, M>);

    /// The number of collision detected during the last update.
    fn num_colls(&self) -> uint;

    /// Collects the collisions detected during the last update.
    fn colls(&self, &mut Vec<Contact<N, P, V>>);
}

/// Trait to be implemented by collision detector using dynamic dispatch.
///
/// This is used to know the exact type of the geometries.
pub trait DynamicCollisionDetector<N, P, V, M, I, G1, G2>: ShapeShapeCollisionDetector<N, P, V, M, I> { }

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

impl<N, P, V, M, D, G1, G2> CollisionDetector<N, P, V, M, G1, G2> for DetectorWithoutRedispatch<D>
    where D: CollisionDetector<N, P, V, M, G1, G2> {
    fn update(&mut self, _: &M, _: &G1, _: &M, _: &G2) { unreachable!() }
    fn num_colls(&self) -> uint { unreachable!() }
    fn colls(&self, _: &mut Vec<Contact<N, P, V>>) { unreachable!() }
}

impl<N, P, V, M, I, D: CollisionDetector<N, P, V, M, G1, G2>, G1, G2>
DynamicCollisionDetector<N, P, V, M, I, G1, G2> for DetectorWithoutRedispatch<D> { }

impl<N, P, V, M, I, D, G1, G2> ShapeShapeCollisionDetector<N, P, V, M, I> for DetectorWithoutRedispatch<D>
    where D:  CollisionDetector<N, P, V, M, G1, G2>,
          G1: 'static,
          G2: 'static {
    #[inline]
    fn update(&mut self,
              _:  &ShapeShapeDispatcher<N, P, V, M, I>,
              m1: &M,
              g1: &Shape<N, P, V, M>,
              m2: &M,
              g2: &Shape<N, P, V, M>) {
        self.detector.update(
            m1,
            g1.downcast_ref::<G1>().expect("Invalid shape."),
            m2,
            g2.downcast_ref::<G2>().expect("Invalid shape."))
    }

    #[inline]
    fn num_colls(&self) -> uint {
        self.detector.num_colls()
    }

    #[inline]
    fn colls(&self, cs: &mut Vec<Contact<N, P, V>>) {
        self.detector.colls(cs)
    }
}

/// Collision dispatcher between two `~Shape`.
pub struct ShapeShapeDispatcher<N, P, V, M, I> {
    constructors: HashMap<(TypeId, TypeId), Box<CollisionDetectorFactory<N, P, V, M, I>>>
}

impl<N, P, V, M, I> ShapeShapeDispatcher<N, P, V, M, I> {
    /// Creates a new `ShapeShapeDispatcher` without the default set of collision detectors
    /// factories.
    pub fn new_without_default() -> ShapeShapeDispatcher<N, P, V, M, I> {
        ShapeShapeDispatcher {
            constructors: HashMap::new()
        }
    }

    /// Registers a new collision detection algorithm factory for a pair of geometries.
    ///
    /// This is unsafe because there is no way to check that the factory will really generate
    /// collision detectors suited for `G1` and `G2`. Whenever possible, use `register_detector` or
    /// `register_dynamic_detector` instead.
    pub unsafe fn register_factory<G1, G2, F>(&mut self, factory: F)
        where G1: 'static + Any,
              G2: 'static + Any,
              F:  CollisionDetectorFactory<N, P, V, M, I> {
        let key = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.insert(key, box factory as Box<CollisionDetectorFactory<N, P, V, M, I>>);
    }

    /// Registers a new dynamic collision detector for two geometries.
    pub fn register_dynamic_detector<G1, G2, D>(&mut self, d: D)
        where G1: 'static + Any,
              G2: 'static + Any,
              D:  'static + Send + DynamicCollisionDetector<N, P, V, M, I, G1, G2> + Clone {
        let factory = CollisionDetectorCloner::new(d);
        unsafe { self.register_factory::<G1, G2, _>(factory) }
    }

    /// Registers a new collision detector for two geometries.
    pub fn register_detector<G1, G2, D>(&mut self, d: D)
        where G1: 'static + Any,
              G2: 'static + Any,
              D:  'static + Send + CollisionDetector<N, P, V, M, G1, G2> + Clone {
        self.register_dynamic_detector(DetectorWithoutRedispatch::new(d));
    }

    /// Unregister the collision detector for a givem pair of geometries.
    pub fn unregister_detector<G1: 'static + Any, G2: 'static + Any>(&mut self) {
        let key = (TypeId::of::<G1>(), TypeId::of::<G2>());
        self.constructors.remove(&key);
    }

    /// If registered, creates a new collision detector adapted for the two given geometries.
    pub fn dispatch(&self, a: &Shape<N, P, V, M>, b: &Shape<N, P, V, M>) -> Option<Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>> {
        self.constructors.find(&(a.get_type_id(), b.get_type_id())).map(|f| f.build())
    }
}

// FIXME: Remove the `UniformSphereSample` bound when the EPA is implemented.
impl<N, P, V, AV, M, I> ShapeShapeDispatcher<N, P, V, M, I>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          I:  Send + Clone {
    // FIXME: make this a function which has the simplex and the prediction margin as parameters
    /// Creates a new `ShapeShapeDispatcher` able do build collision detectors for any valid pair of
    /// geometries supported by `ncollide`.
    pub fn new(prediction: N) -> ShapeShapeDispatcher<N, P, V, M, I> {
        let mut res: ShapeShapeDispatcher<N, P, V, M, I> = ShapeShapeDispatcher::new_without_default();

        // Ball vs. Ball
        let bb = BallBall::new(prediction.clone());
        res.register_detector(bb);

        // Ball vs Bézier
        let selector = HyperPlaneSurfaceSelector::new(Bounded::max_value());
        let bs       = BallBezierSurface::new(selector.clone(), prediction.clone());
        let sb       = BezierSurfaceBall::new(selector.clone(), prediction.clone());
        res.register_detector(bs);
        res.register_detector(sb);

        // Plane vs. SupportMap
        res.register_default_plane_implicit_detector::<Ball<N>>(false, prediction);
        res.register_default_plane_implicit_detector::<Cuboid<V>>(true, prediction);
        res.register_default_plane_implicit_detector::<Cone<N>>(true, prediction);
        res.register_default_plane_implicit_detector::<Cylinder<N>>(true, prediction);
        res.register_default_plane_implicit_detector::<Capsule<N>>(true, prediction);
        res.register_default_plane_implicit_detector::<Convex<P>>(true, prediction);
        res.register_default_plane_implicit_detector::<Triangle<P>>(true, prediction);
        res.register_default_plane_implicit_detector::<Segment<P>>(true, prediction);

        // SupportMap vs. SupportMap
        // NOTE: some pair will be registered twice…
        res.register_default_implicit_detectors::<Cuboid<V>>(true, prediction);
        res.register_default_implicit_detectors::<Cone<N>>(true, prediction);
        res.register_default_implicit_detectors::<Cylinder<N>>(true, prediction);
        res.register_default_implicit_detectors::<Capsule<N>>(true, prediction);
        res.register_default_implicit_detectors::<Convex<P>>(true, prediction);
        res.register_default_implicit_detectors::<Triangle<P>>(true, prediction);
        res.register_default_implicit_detectors::<Segment<P>>(true, prediction);

        // FIXME: refactor the three following blocks?
        // Compound vs. Other
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Plane<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Ball<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Cuboid<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Cone<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Cylinder<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Capsule<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Convex<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Triangle<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Segment<P>>(prediction);

        // TriangleMesh vs. Other
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Plane<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Ball<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Cuboid<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Cone<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Cylinder<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Capsule<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Convex<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Triangle<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Segment<P>>(prediction);

        // LineStrip vs. Other
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Plane<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Ball<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Cuboid<V>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Cone<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Cylinder<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Capsule<N>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Convex<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Triangle<P>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Segment<P>>(prediction);

        // // FIXME: implement a ConcaveShapeConcaveShape detector?
        res.register_default_concave_geom_geom_detector::<Compound<N, P, V, M, I>, Compound<N, P, V, M, I>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Segment<P>>, Compound<N, P, V, M, I>>(prediction);
        res.register_default_concave_geom_geom_detector::<Mesh<N, P, V, Triangle<P>>, Compound<N, P, V, M, I>>(prediction);

        res
    }

    /// Registers a `PlaneSupportMap` collision detector between a given support mapped shape and a plane.
    pub fn register_default_plane_implicit_detector<I>(&mut self, generate_manifold: bool, prediction: N)
        where I: 'static + SupportMap<P, V, M> {
        let d1 = SupportMapPlane::<N, P, V, I>::new(prediction.clone());
        let d2 = PlaneSupportMap::<N, P, V, I>::new(prediction.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    /// Register an `SupportMapSupportMap` collision detector between two implicit geometries.
    pub fn register_default_implicit_implicit_detector<G1, G2, S>(&mut self,
                                                                  generate_manifold: bool,
                                                                  prediction:        N,
                                                                  simplex:           &S)
        where G1: 'static + SupportMap<P, V, M> + PreferedSamplingDirections<V, M>,
              G2: 'static + SupportMap<P, V, M> + PreferedSamplingDirections<V, M>,
              S:  Send + Simplex<N, AnnotatedPoint<P>> + Clone {
        let d1 = SupportMapSupportMap::<N, P, V, S, G1, G2>::new(prediction.clone(), simplex.clone());
        let d2 = SupportMapSupportMap::<N, P, V, S, G2, G1>::new(prediction.clone(), simplex.clone());

        if generate_manifold {
            self.register_detector_with_contact_manifold_generator(d1, prediction);
            self.register_detector_with_contact_manifold_generator(d2, prediction);
        }
        else {
            self.register_detector(d1);
            self.register_detector(d2);
        }
    }

    /// Register an `ConcaveShapeShape` collision detector between a given concave shape and a
    /// given shape.
    pub fn register_default_concave_geom_geom_detector<G1, G2>(&mut self, prediction: N)
        where G1: 'static + ConcaveShape<N, P, V, M>,
              G2: 'static + Shape<N, P, V, M> {
        let  f1 = ConcaveShapeShapeFactory::<N, P, V, M, G1, G2>::new(prediction.clone());
        let  f2 = ShapeConcaveShapeFactory::<N, P, V, M, G2, G1>::new(prediction.clone());

        // FIXME: find a way to factorize that?
        unsafe { self.register_factory::<G1, G2, _>(f1) }
        unsafe { self.register_factory::<G2, G1, _>(f2) }
    }

    // XXX: improve this when conditional dispatch is added to rustc so that we use the
    // OneShotContactManifoldGenerator when `Cross` is implemented.
    /// Register a given collision detector and adds it a contact manifold generator (a
    /// `OneShotContactManifoldGenerator`).
    pub fn register_detector_with_contact_manifold_generator<G1, G2, D>(&mut self, d: D, prediction: N)
        where G1: 'static + Any,
              G2: 'static + Any,
              D:  'static + Send + CollisionDetector<N, P, V, M, G1, G2> + Clone {
        let d = OSCMG::<N, P, V, D>::new(prediction.clone(), d);
        self.register_detector(d);
    }

    /// Register `SupportMapSupportMap` collision detectors between a given shape and every implicit
    /// shape supported by `ncollide`.
    pub fn register_default_implicit_detectors<G>(&mut self, generate_manifold: bool, prediction: N)
        where G: 'static + SupportMap<P, V, M> + PreferedSamplingDirections<V, M> {

        type S<N, P, V>  = JohnsonSimplex<N, AnnotatedPoint<P>, V>;

        // SupportMap vs. SupportMap
        let rt = RecursionTemplate::new(na::dim::<V>());
        let js = &JohnsonSimplex::new(rt);

        self.register_default_implicit_implicit_detector::<Ball<N>, G, S<N, P, V>>(false, prediction, js);
        self.register_default_implicit_implicit_detector::<Cuboid<V>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cone<N>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Cylinder<N>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Capsule<N>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Convex<P>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Triangle<P>, G, S<N, P, V>>(generate_manifold, prediction, js);
        self.register_default_implicit_implicit_detector::<Segment<P>, G, S<N, P, V>>(generate_manifold, prediction, js);
    }
}

// FIXME: rename that ShapeShapeCollisionDetectorFactory ?
/// Trait of structures able do build a new collision detector.
pub trait CollisionDetectorFactory<N, P, V, M, I> : Send {
    /// Builds a new collision detector.
    fn build(&self) -> Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>;
}

/// Cloning-based collision detector factory.
pub struct CollisionDetectorCloner<CD> {
    template: CD
}

impl<N, P, V, M, I, CD: ShapeShapeCollisionDetector<N, P, V, M, I> + Clone> CollisionDetectorCloner<CD> {
    /// Creates a new `CollisionDetectorCloner`.
    ///
    /// The cloned detector is `CD`.
    fn new(detector: CD) -> CollisionDetectorCloner<CD> {
        CollisionDetectorCloner {
            template: detector
        }
    }
}

impl<N, P, V, M, I, CD> CollisionDetectorFactory<N, P, V, M, I> for CollisionDetectorCloner<CD>
    where CD: 'static + Send + ShapeShapeCollisionDetector<N, P, V, M, I> + Clone {
    fn build(&self) -> Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send> {
        box self.template.clone() as Box<ShapeShapeCollisionDetector<N, P, V, M, I> + Send>
    }
}
