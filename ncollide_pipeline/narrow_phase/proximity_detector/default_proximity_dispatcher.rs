use std::ops::Mul;
use std::marker::PhantomData;
use na::{Translate, Cross, Translation, Rotation};
use math::{Point, Vect, Isometry};
use entities::inspection;
use entities::inspection::ReprDesc;
use entities::shape::{Ball, Plane};
use queries::geometry::algorithms::johnson_simplex::JohnsonSimplex;
use narrow_phase::proximity_detector::{
    ProximityDispatcher,
    ProximityAlgorithm,
    BallBallProximityDetector,
    PlaneSupportMapProximityDetector,
    SupportMapPlaneProximityDetector,
    SupportMapSupportMapProximityDetector,
    CompositeShapeReprProximityDetector,
    ReprCompositeShapeProximityDetector,
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
          <P::Vect as Cross>::CrossProductType: Vect<Scalar = <P::Vect as Vect>::Scalar> +
                                                Mul<<P::Vect as Vect>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M: Isometry<P, P::Vect> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    fn get_proximity_algorithm(&self, a: &ReprDesc<P, M>, b: &ReprDesc<P, M>) -> Option<ProximityAlgorithm<P, M>> {
        let a_is_ball = a.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>().is_some();
        let b_is_ball = b.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>().is_some();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallProximityDetector::<P, M>::new()))
        }
        else if a.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
            Some(Box::new(PlaneSupportMapProximityDetector::<P, M>::new()))
        }
        else if b.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() {
            Some(Box::new(SupportMapPlaneProximityDetector::<P, M>::new()))
        }
        else if inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
            let simplex = JohnsonSimplex::new_w_tls();
            Some(Box::new(SupportMapSupportMapProximityDetector::new(simplex)))
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*a).is_some() {
            Some(Box::new(CompositeShapeReprProximityDetector::<P, M>::new()))
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*b).is_some() {
            Some(Box::new(ReprCompositeShapeProximityDetector::<P, M>::new()))
        }
        else {
            None
        }
    }
}
