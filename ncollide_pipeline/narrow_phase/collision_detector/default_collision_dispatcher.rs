use std::ops::Mul;
use std::marker::PhantomData;
use na::{Translate, Cross, Translation, Rotation};
use math::{Point, Vector, Isometry};
use geometry::shape::{Shape, Ball, Plane};
use geometry::query::algorithms::johnson_simplex::JohnsonSimplex;
use narrow_phase::{
    CollisionDispatcher,
    CollisionAlgorithm,
    BallBallCollisionDetector,
    PlaneSupportMapCollisionDetector,
    SupportMapPlaneCollisionDetector,
    SupportMapSupportMapCollisionDetector,
    CompositeShapeShapeCollisionDetector,
    ShapeCompositeShapeCollisionDetector,
    OneShotContactManifoldGenerator
};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct DefaultCollisionDispatcher<P: Point, M> {
    _point_type:  PhantomData<P>,
    _matrix_type: PhantomData<M>,
}

impl<P: Point, M> DefaultCollisionDispatcher<P, M> {
    /// Creates a new basic collision dispatcher.
    pub fn new() -> DefaultCollisionDispatcher<P, M> {
        DefaultCollisionDispatcher {
            _point_type:  PhantomData,
            _matrix_type: PhantomData,
        }
    }
}

impl<P, M> CollisionDispatcher<P, M> for DefaultCollisionDispatcher<P, M>
    where P: Point,
          P::Vect: Translate<P> + Cross,
          <P::Vect as Cross>::CrossProductType: Vector<Scalar = <P::Vect as Vector>::Scalar> +
                                                Mul<<P::Vect as Vector>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M: Isometry<P> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    fn get_collision_algorithm(&self, a: &Shape<P, M>, b: &Shape<P, M>) -> Option<CollisionAlgorithm<P, M>> {
        let a_is_ball = a.is_shape::<Ball<<P::Vect as Vector>::Scalar>>();
        let b_is_ball = b.is_shape::<Ball<<P::Vect as Vector>::Scalar>>();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallCollisionDetector::<P, M>::new()))
        }
        else if a.is_shape::<Plane<P::Vect>>() && b.is_support_map() {
            let wo_manifold = PlaneSupportMapCollisionDetector::<P, M>::new();

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if b.is_shape::<Plane<P::Vect>>() && a.is_support_map() {
            let wo_manifold = SupportMapPlaneCollisionDetector::<P, M>::new();

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if a.is_support_map() && b.is_support_map() {
            let simplex = JohnsonSimplex::new_w_tls();
            let wo_manifold = SupportMapSupportMapCollisionDetector::new(simplex);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if a.is_composite_shape() {
            Some(Box::new(CompositeShapeShapeCollisionDetector::<P, M>::new()))
        }
        else if b.is_composite_shape() {
            Some(Box::new(ShapeCompositeShapeCollisionDetector::<P, M>::new()))
        }
        else {
            None
        }
    }
}
