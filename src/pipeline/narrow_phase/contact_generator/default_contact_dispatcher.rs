use std::marker::PhantomData;
use na::Real;
use math::{Isometry, Point};
use shape::{Ball, Plane, Shape};
use query::algorithms::VoronoiSimplex;
use pipeline::narrow_phase::{BallBallManifoldGenerator,
                   BallConvexPolyhedronManifoldGenerator,
                   CompositeShapeShapeManifoldGenerator,
                   ContactAlgorithm,
                   ContactDispatcher,
                   ConvexPolyhedronConvexPolyhedronManifoldGenerator,
                   PlaneBallManifoldGenerator,
                   PlaneConvexPolyhedronManifoldGenerator};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher {
}

impl DefaultContactDispatcher {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher {
        DefaultContactDispatcher {
        }
    }
}

impl<N: Real> ContactDispatcher<N> for DefaultContactDispatcher {
    fn get_contact_algorithm(
        &self,
        a: &Shape<N>,
        b: &Shape<N>,
    ) -> Option<ContactAlgorithm<N>> {
        let a_is_ball = a.is_shape::<Ball<N>>();
        let b_is_ball = b.is_shape::<Ball<N>>();
        let a_is_plane = a.is_shape::<Plane<N>>();
        let b_is_plane = b.is_shape::<Plane<N>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallManifoldGenerator::<N>::new()))
        } else if a_is_plane && b_is_ball {
            Some(Box::new(PlaneBallManifoldGenerator::<N>::new(false)))
        } else if a_is_ball && b_is_plane {
            Some(Box::new(PlaneBallManifoldGenerator::<N>::new(true)))
        } else if a_is_plane && b.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(Box::new(gen))
        } else if b_is_plane && a.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(Box::new(gen))
        } else if a_is_ball && b.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(false);
            Some(Box::new(gen))
        } else if b_is_ball && a.is_convex_polyhedron() {
            let gen = BallConvexPolyhedronManifoldGenerator::<N>::new(true);
            Some(Box::new(gen))
        } else if a.is_convex_polyhedron() && b.is_convex_polyhedron() {
            let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new();
            Some(Box::new(gen))
        } else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeManifoldGenerator::<N>::new(
                false,
            )))
        } else if b.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeManifoldGenerator::<N>::new(
                true,
            )))
        } else {
            None
        }
    }
}
