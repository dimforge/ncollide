use std::marker::PhantomData;
use na;
use math::{Isometry, Point};
use shape::{Ball, Plane, Shape};
use query::algorithms::{VoronoiSimplex, VoronoiSimplex};
use pipeline::narrow_phase::{BallBallManifoldGenerator,
                   BallConvexPolyhedronManifoldGenerator,
                   CompositeShapeShapeManifoldGenerator,
                   ContactAlgorithm,
                   ContactDispatcher, // OneShotContactManifoldGenerator,
                   ConvexPolyhedronConvexPolyhedronManifoldGenerator,
                   PlaneBallManifoldGenerator,
                   PlaneConvexPolyhedronManifoldGenerator};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher<N> {
    _point_type: PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<N> DefaultContactDispatcher<N> {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher<N> {
        DefaultContactDispatcher {
            _point_type: PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<N: Real> ContactDispatcher<N> for DefaultContactDispatcher<N> {
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
            match na::dimension::<Vector<N>>() {
                2 => {
                    let simplex = VoronoiSimplex::new();
                    // let simplex = JohnsonSimplex::new_w_tls();

                    let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new(simplex);
                    Some(Box::new(gen))
                }
                3 => {
                    let simplex = VoronoiSimplex::new();
                    // let simplex = JohnsonSimplex::new_w_tls();

                    let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new(simplex);
                    Some(Box::new(gen))
                }
                _ => unimplemented!(),
            }
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
