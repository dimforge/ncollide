use std::ops::Mul;
use na::{Translate, Cross, Translation, Rotation};
use math::{Scalar, Point, Vect, Isometry};
use entities::inspection;
use entities::inspection::ReprDesc;
use entities::shape::{Ball, Plane};
use queries::geometry::algorithms::johnson_simplex::JohnsonSimplex;
use narrow_phase::{
    CollisionDispatcher,
    CollisionAlgorithm,
    BallBall,
    PlaneSupportMap,
    SupportMapPlane,
    SupportMapSupportMap,
    CompositeShapeRepr,
    ReprCompositeShape,
    OneShotContactManifoldGenerator
};

/// Collision dispatcher for shapes defined by `ncollide_entities`.
pub struct BasicCollisionDispatcher<N> {
    prediction: N
}

impl<N> BasicCollisionDispatcher<N> {
    /// Creates a new basic collision dispatcher.
    pub fn new(prediction: N) -> BasicCollisionDispatcher<N> {
        BasicCollisionDispatcher {
            prediction: prediction
        }
    }
}

impl<P, M> CollisionDispatcher<P, M> for BasicCollisionDispatcher<<P::Vect as Vect>::Scalar>
    where P: Point,
          P::Vect: Translate<P> + Cross,
          <P::Vect as Cross>::CrossProductType: Vect<Scalar = <P::Vect as Vect>::Scalar> +
                                                Mul<<P::Vect as Vect>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M: Isometry<P, P::Vect> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    fn get_collision_algorithm(&self, a: &ReprDesc, b: &ReprDesc) -> Option<CollisionAlgorithm<P, M>> {
        let a_is_ball = a.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>().is_some();
        let b_is_ball = b.downcast_ref::<Ball<<P::Vect as Vect>::Scalar>>().is_some();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBall::<P, M>::new(self.prediction)))
        }
        else if a.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
            let wo_manifold = PlaneSupportMap::<P, M>::new(self.prediction);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if b.downcast_ref::<Plane<P::Vect>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() {
            let wo_manifold = SupportMapPlane::<P, M>::new(self.prediction);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if inspection::maybe_repr_desc_as_support_map::<P, M>(*a).is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, M>(*b).is_some() {
            let simplex = JohnsonSimplex::new_w_tls();
            let wo_manifold = SupportMapSupportMap::new(self.prediction, simplex);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*a).is_some() {
            Some(Box::new(CompositeShapeRepr::<P, M>::new(self.prediction)))
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<P, M>(*b).is_some() {
            Some(Box::new(ReprCompositeShape::<P, M>::new(self.prediction)))
        }
        else {
            None
        }
    }
}
