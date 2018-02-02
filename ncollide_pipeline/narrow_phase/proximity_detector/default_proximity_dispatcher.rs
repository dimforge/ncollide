use std::marker::PhantomData;
use math::{Point, Isometry};
use na;
use geometry::shape::{Shape, Ball, Plane};
use geometry::query::algorithms::johnson_simplex::JohnsonSimplex;
use geometry::query::algorithms::voronoi_simplex2::VoronoiSimplex2;
use geometry::query::algorithms::voronoi_simplex3::VoronoiSimplex3;
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

impl<P: Point, M: Isometry<P>> ProximityDispatcher<P, M> for DefaultProximityDispatcher<P, M> {
    fn get_proximity_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<ProximityAlgorithm<P, M>> {
        let a_is_ball = a.is_shape::<Ball<P::Real>>();
        let b_is_ball = b.is_shape::<Ball<P::Real>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallProximityDetector::<P, M>::new()))
        }
        else if a.is_shape::<Plane<P::Vector>>() && b.is_support_map() {
            Some(Box::new(PlaneSupportMapProximityDetector::<P, M>::new()))
        }
        else if b.is_shape::<Plane<P::Vector>>() && a.is_support_map() {
            Some(Box::new(SupportMapPlaneProximityDetector::<P, M>::new()))
        }
        else if a.is_support_map() && b.is_support_map() {
            if na::dimension::<P::Vector>() == 2 {
                let simplex = VoronoiSimplex2::new();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(simplex)))
            }
            else if na::dimension::<P::Vector>() == 3 {
                let simplex = VoronoiSimplex3::new();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(simplex)))
            }
            else {
                let simplex = JohnsonSimplex::new_w_tls();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(simplex)))
            }
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
