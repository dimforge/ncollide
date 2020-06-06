use crate::math::Isometry;
use crate::pipeline::narrow_phase::{
    ContactDispatcher, ContactManifoldGenerator, ConvexPolyhedronConvexPolyhedronManifoldGenerator,
};
use crate::query::{ContactManifold, ContactPrediction, ContactPreprocessor};
use crate::shape::{Capsule, Shape};
use na::{self, RealField};

/// Collision detector between a concave shape and another shape.
pub struct CapsuleCapsuleManifoldGenerator<N: RealField> {
    // FIXME: use a dedicated segment-segment algorithm instead.
    sub_detector: ConvexPolyhedronConvexPolyhedronManifoldGenerator<N>,
}

impl<N: RealField> CapsuleCapsuleManifoldGenerator<N> {
    /// Creates a new collision detector between a concave shape and another shape.
    pub fn new() -> CapsuleCapsuleManifoldGenerator<N> {
        CapsuleCapsuleManifoldGenerator {
            sub_detector: ConvexPolyhedronConvexPolyhedronManifoldGenerator::new(),
        }
    }

    fn do_update(
        &mut self,
        dispatcher: &dyn ContactDispatcher<N>,
        m1: &Isometry<N>,
        g1: &Capsule<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        g2: &Capsule<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        let segment1 = g1.segment();
        let segment2 = g2.segment();

        let mut prediction = prediction.clone();
        let new_linear_prediction = prediction.linear() + g1.radius + g2.radius;
        prediction.set_linear(new_linear_prediction);

        // Update all collisions
        self.sub_detector.generate_contacts(
            dispatcher,
            m1,
            &segment1,
            Some(&(proc1, &g1.contact_preprocessor())),
            m2,
            &segment2,
            Some(&(proc2, &g2.contact_preprocessor())),
            &prediction,
            manifold,
        )
    }
}

impl<N: RealField> ContactManifoldGenerator<N> for CapsuleCapsuleManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        d: &dyn ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if let (Some(cs1), Some(cs2)) = (a.as_shape::<Capsule<N>>(), b.as_shape::<Capsule<N>>()) {
            self.do_update(d, ma, cs1, proc1, mb, cs2, proc2, prediction, manifold)
        } else {
            false
        }
    }
}
