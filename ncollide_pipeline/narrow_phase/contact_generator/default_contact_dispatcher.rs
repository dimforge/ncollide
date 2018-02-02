use std::marker::PhantomData;
use na;
use math::{Point, Isometry};
use geometry::shape::{Shape, Ball, Plane};
use geometry::query::algorithms::johnson_simplex::JohnsonSimplex;
use geometry::query::algorithms::voronoi_simplex2::VoronoiSimplex2;
use geometry::query::algorithms::voronoi_simplex3::VoronoiSimplex3;
use narrow_phase::{
    ContactDispatcher,
    ContactAlgorithm,
    BallBallContactGenerator,
    PlaneSupportMapContactGenerator,
    SupportMapPlaneContactGenerator,
    SupportMapSupportMapContactGenerator,
    CompositeShapeShapeContactGenerator,
    ShapeCompositeShapeContactGenerator,
    OneShotContactManifoldGenerator
};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultContactDispatcher<P: Point, M> {
    _point_type:  PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<P: Point, M> DefaultContactDispatcher<P, M> {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultContactDispatcher<P, M> {
        DefaultContactDispatcher {
            _point_type:  PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactDispatcher<P, M> for DefaultContactDispatcher<P, M> {
    fn get_contact_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<ContactAlgorithm<P, M>> {
        let a_is_ball = a.is_shape::<Ball<P::Real>>();
        let b_is_ball = b.is_shape::<Ball<P::Real>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallContactGenerator::<P, M>::new()))
        }
        else if a.is_shape::<Plane<P::Vector>>() && b.is_support_map() {
            let wo_manifold = PlaneSupportMapContactGenerator::<P, M>::new();

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if b.is_shape::<Plane<P::Vector>>() && a.is_support_map() {
            let wo_manifold = SupportMapPlaneContactGenerator::<P, M>::new();

            if !a_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if a.is_support_map() && b.is_support_map() {
            match na::dimension::<P::Vector>() {
                2 => {
                    let simplex     = VoronoiSimplex2::new();
                    let wo_manifold = SupportMapSupportMapContactGenerator::new(simplex);

                    if !a_is_ball && !b_is_ball {
                        let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                        Some(Box::new(manifold))
                    }
                    else {
                        Some(Box::new(wo_manifold))
                    }
                },
                3 => {
                    let simplex     = VoronoiSimplex3::new();
                    let wo_manifold = SupportMapSupportMapContactGenerator::new(simplex);

                    if !a_is_ball && !b_is_ball {
                        let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                        Some(Box::new(manifold))
                    }
                    else {
                        Some(Box::new(wo_manifold))
                    }
                },
                _ => {
                    let simplex     = JohnsonSimplex::new_w_tls();
                    let wo_manifold = SupportMapSupportMapContactGenerator::new(simplex);

                    if false { // !a_is_ball && !b_is_ball {
                        let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                        Some(Box::new(manifold))
                    }
                    else {
                        Some(Box::new(wo_manifold))
                    }
                }
            }

        }
        else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeContactGenerator::<P, M>::new()))
        }
        else if b.is_composite_shape() {
            Some(Box::new(ShapeCompositeShapeContactGenerator::<P, M>::new()))
        }
        else {
            None
        }
    }
}
