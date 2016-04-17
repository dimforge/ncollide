use std::ops::Mul;
use std::marker::PhantomData;
use na::{Translate, Cross, Translation, Rotation};
use math::{Point, Vector, Isometry};
use entities::inspection;
use entities::inspection::ReprDesc;
use entities::shape::{Ball, Plane};
use queries::geometry::algorithms::johnson_simplex::JohnsonSimplex;
use narrow_phase::{
    CollisionDispatcher,
    CollisionAlgorithm,
    BallBallCollisionDetector,
    PlaneSupportMapCollisionDetector,
    SupportMapPlaneCollisionDetector,
    SupportMapSupportMapCollisionDetector,
    CompositeShapeReprCollisionDetector,
    ReprCompositeShapeCollisionDetector,
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
          M: Isometry<P, P::Vect> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    fn get_collision_algorithm(&self, a: &ReprDesc<P, M>, b: &ReprDesc<P, M>) -> Option<CollisionAlgorithm<P, M>> {
        let a_is_ball = a.downcast_ref::<Ball<<P::Vect as Vector>::Scalar>>().is_some();
        let b_is_ball = b.downcast_ref::<Ball<<P::Vect as Vector>::Scalar>>().is_some();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBallCollisionDetector::<P, M>::new()))
        }
        else if a.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
            let wo_manifold = PlaneSupportMapCollisionDetector::<P, M>::new();

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if b.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() {
            let wo_manifold = SupportMapPlaneCollisionDetector::<P, M>::new();

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
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
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*a).is_some() {
            Some(Box::new(CompositeShapeReprCollisionDetector::<P, M>::new()))
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*b).is_some() {
            Some(Box::new(ReprCompositeShapeCollisionDetector::<P, M>::new()))
        }
        else {
            None
        }
    }
}
