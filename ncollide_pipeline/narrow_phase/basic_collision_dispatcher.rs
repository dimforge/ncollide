use na::{Translate, Cross, Rotation};
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

impl<N, P, V, AV, M> CollisionDispatcher<N, P, V, M> for BasicCollisionDispatcher<N>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<Output = AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV> {
    fn get_collision_algorithm(&self, a: &ReprDesc, b: &ReprDesc) -> Option<CollisionAlgorithm<N, P, V, M>> {
        let a_is_ball = a.downcast_ref::<Ball<N>>().is_some();
        let b_is_ball = b.downcast_ref::<Ball<N>>().is_some();

        if a_is_ball && b_is_ball {
            Some(Box::new(BallBall::<N, P, V, M>::new(self.prediction)))
        }
        else if a.downcast_ref::<Plane<V>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, V, M>(*b).is_some() {
            let wo_manifold = PlaneSupportMap::<N, P, V, M>::new(self.prediction);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if b.downcast_ref::<Plane<V>>().is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, V, M>(*a).is_some() {
            let wo_manifold = SupportMapPlane::<N, P, V, M>::new(self.prediction);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if inspection::maybe_repr_desc_as_support_map::<P, V, M>(*a).is_some() &&
                inspection::maybe_repr_desc_as_support_map::<P, V, M>(*b).is_some() {
            let simplex = JohnsonSimplex::<N, _, V>::new_w_tls();
            let wo_manifold = SupportMapSupportMap::<N, P, V, M, _>::new(self.prediction, simplex);

            if !b_is_ball {
                let manifold = OneShotContactManifoldGenerator::new(self.prediction, wo_manifold);
                Some(Box::new(manifold))
            }
            else {
                Some(Box::new(wo_manifold))
            }
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<N, P, V, M>(*a).is_some() {
            Some(Box::new(CompositeShapeRepr::<N, P, V, M>::new(self.prediction)))
        }
        else if inspection::maybe_repr_desc_as_composite_shape::<N, P, V, M>(*b).is_some() {
            Some(Box::new(ReprCompositeShape::<N, P, V, M>::new(self.prediction)))
        }
        else {
            None
        }
    }
}
