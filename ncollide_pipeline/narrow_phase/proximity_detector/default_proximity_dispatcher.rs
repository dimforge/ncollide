use std::ops::Mul;
use std::marker::PhantomData;
use na::{Translate, Cross, Translation, Rotation};
use math::{Point, Vector, Isometry};
use geometry::shape::{Shape, Ball, Plane};
use geometry::query::algorithms::johnson_simplex::JohnsonSimplex;
use narrow_phase::proximity_detector::{
    ProximityDispatcher,
    ProximityAlgorithm,
    BallBallProximityDetector,
    PlaneSupportMapProximityDetector,
    SupportMapPlaneProximityDetector,
    SupportMapSupportMapProximityDetector,
    CompositeShapeShapeProximityDetector,
    ShapeCompositeShapeProximityDetector,
};

/// Proximity dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultProximityDispatcher<P: Point, M> {
    _point_type:  PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<P: Point, M> DefaultProximityDispatcher<P, M> {
    /// Creates a new basic proximity dispatcher.
    pub fn new() -> DefaultProximityDispatcher<P, M> {
        DefaultProximityDispatcher {
            _point_type:  PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<P, M> ProximityDispatcher<P, M> for DefaultProximityDispatcher<P, M>
    where P: Point,
          P::Vect: Translate<P> + Cross,
          <P::Vect as Cross>::CrossProductType: Vector<Scalar = <P::Vect as Vector>::Scalar> +
                                                Mul<<P::Vect as Vector>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M: Isometry<P> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    fn get_proximity_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<ProximityAlgorithm<P, M>> {
        let a_is_ball = a.is_shape::<Ball<<P::Vect as Vector>::Scalar>>();
        let b_is_ball = b.is_shape::<Ball<<P::Vect as Vector>::Scalar>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallProximityDetector::<P, M>::new()))
        }
        else if a.is_shape::<Plane<P::Vect>>() && b.is_support_map() {
            Some(Box::new(PlaneSupportMapProximityDetector::<P, M>::new()))
        }
        else if b.is_shape::<Plane<P::Vect>>() && a.is_support_map() {
            Some(Box::new(SupportMapPlaneProximityDetector::<P, M>::new()))
        }
        else if a.is_support_map() && b.is_support_map() {
            let simplex = JohnsonSimplex::new_w_tls();
            Some(Box::new(SupportMapSupportMapProximityDetector::new(simplex)))
        }
        else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeProximityDetector::<P, M>::new()))
        }
        else if b.is_composite_shape() {
            Some(Box::new(ShapeCompositeShapeProximityDetector::<P, M>::new()))
        }
        else {
            None
        }
    }
}
