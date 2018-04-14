use std::marker::PhantomData;
use math::{Isometry, Point};
use na;
use shape::{Ball, Plane, Shape};
use query::algorithms::{JohnsonSimplex, VoronoiSimplex2, VoronoiSimplex3};
use pipeline::narrow_phase::proximity_detector::{BallBallProximityDetector,
                                                CompositeShapeShapeProximityDetector,
                                                PlaneSupportMapProximityDetector, ProximityAlgorithm,
                                                ProximityDispatcher, ShapeCompositeShapeProximityDetector,
                                                SupportMapPlaneProximityDetector,
                                                SupportMapSupportMapProximityDetector};

/// Proximity dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultProximityDispatcher<N> {
    _point_type: PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<N> DefaultProximityDispatcher<N> {
    /// Creates a new basic proximity dispatcher.
    pub fn new() -> DefaultProximityDispatcher<N> {
        DefaultProximityDispatcher {
            _point_type: PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<N: Real> ProximityDispatcher<N> for DefaultProximityDispatcher<N> {
    fn get_proximity_algorithm(
        &self,
        a: &Shape<N>,
        b: &Shape<N>,
    ) -> Option<ProximityAlgorithm<N>> {
        let a_is_ball = a.is_shape::<Ball<N>>();
        let b_is_ball = b.is_shape::<Ball<N>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallProximityDetector::<N>::new()))
        } else if a.is_shape::<Plane<N>>() && b.is_support_map() {
            Some(Box::new(PlaneSupportMapProximityDetector::<N>::new()))
        } else if b.is_shape::<Plane<N>>() && a.is_support_map() {
            Some(Box::new(SupportMapPlaneProximityDetector::<N>::new()))
        } else if a.is_support_map() && b.is_support_map() {
            if na::dimension::<Vector<N>>() == 2 {
                let simplex = VoronoiSimplex2::new();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(
                    simplex,
                )))
            } else if na::dimension::<Vector<N>>() == 3 {
                let simplex = VoronoiSimplex3::new();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(
                    simplex,
                )))
            } else {
                let simplex = JohnsonSimplex::new_w_tls();
                Some(Box::new(SupportMapSupportMapProximityDetector::new(
                    simplex,
                )))
            }
        } else if a.is_composite_shape() {
            Some(Box::new(
                CompositeShapeShapeProximityDetector::<N>::new(),
            ))
        } else if b.is_composite_shape() {
            Some(Box::new(
                ShapeCompositeShapeProximityDetector::<N>::new(),
            ))
        } else {
            None
        }
    }
}
