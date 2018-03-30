use std::marker::PhantomData;
use na;
use math::{Isometry, Point};
use geometry::shape::{Ball, Plane, Shape};
use geometry::query::algorithms::{VoronoiSimplex2, VoronoiSimplex3};
use narrow_phase::{BallBallContactGenerator,
                   CompositeShapeShapeContactGenerator,
                   ContactAlgorithm,
                   ContactDispatcher, // OneShotContactManifoldGenerator,
                   ConvexPolyhedronConvexPolyhedronManifoldGenerator,
                   PlaneBallManifoldGenerator,
                   PlaneConvexPolyhedronManifoldGenerator,
                   ShapeCompositeShapeContactGenerator};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher<P: Point, M> {
    _point_type: PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<P: Point, M> DefaultContactDispatcher<P, M> {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher<P, M> {
        DefaultContactDispatcher {
            _point_type: PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactDispatcher<P, M> for DefaultContactDispatcher<P, M> {
    fn get_contact_algorithm(
        &self,
        a: &Shape<P, M>,
        b: &Shape<P, M>,
    ) -> Option<ContactAlgorithm<P, M>> {
        let a_is_ball = a.is_shape::<Ball<P::Real>>();
        let b_is_ball = b.is_shape::<Ball<P::Real>>();
        let a_is_plane = a.is_shape::<Plane<P::Vector>>();
        let b_is_plane = b.is_shape::<Plane<P::Vector>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallContactGenerator::<P, M>::new()))
        } else if a_is_plane && b_is_ball {
            Some(Box::new(PlaneBallManifoldGenerator::<P, M>::new(false)))
        } else if a_is_ball && b_is_plane {
            Some(Box::new(PlaneBallManifoldGenerator::<P, M>::new(true)))
        } else if a_is_plane && b.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<P, M>::new(false);
            Some(Box::new(gen))
        } else if b_is_plane && a.is_support_map() {
            let gen = PlaneConvexPolyhedronManifoldGenerator::<P, M>::new(true);
            Some(Box::new(gen))
        } else if a.is_convex_polyhedron() && b.is_convex_polyhedron() {
            match na::dimension::<P::Vector>() {
                2 => {
                    let simplex = VoronoiSimplex2::new();
                    // let simplex = JohnsonSimplex::new_w_tls();

                    let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new(simplex);
                    Some(Box::new(gen))
                }
                3 => {
                    let simplex = VoronoiSimplex3::new();
                    // let simplex = JohnsonSimplex::new_w_tls();

                    let gen = ConvexPolyhedronConvexPolyhedronManifoldGenerator::new(simplex);
                    Some(Box::new(gen))
                }
                _ => unimplemented!(),
            }
        } else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeContactGenerator::<P, M>::new()))
        } else if b.is_composite_shape() {
            Some(Box::new(ShapeCompositeShapeContactGenerator::<P, M>::new()))
        } else {
            None
        }
    }
}
