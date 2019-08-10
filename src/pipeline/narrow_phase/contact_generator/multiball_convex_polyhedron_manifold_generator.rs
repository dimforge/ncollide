use crate::math::{Isometry, Point, Translation};
use na::{RealField, Unit};
use crate::pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator, BallConvexPolyhedronManifoldGenerator};
use crate::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction, ContactTrackingMode, NeighborhoodGeometry, ContactPreprocessor};
use crate::shape::{Ball, FeatureId, Shape, Multiball};
use std::marker::PhantomData;
use crate::utils::IsometryOps;

/// Collision detector between two balls.
#[derive(Clone)]
pub struct MultiballConvexPolyhedronManifoldGenerator<N: RealField> {
    phantom: PhantomData<N>,
    flip: bool,
}

impl<N: RealField> MultiballConvexPolyhedronManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> MultiballConvexPolyhedronManifoldGenerator<N> {
        MultiballConvexPolyhedronManifoldGenerator {
            phantom: PhantomData,
            flip,
        }
    }

    fn do_generate(
        &mut self,
        m1: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if let (Some(multiball), Some(pq2), Some(cp2)) = (
            a.as_shape::<Multiball<N>>(),
            b.as_point_query(),
            b.as_convex_polyhedron(),
        ) {
            let generator = BallConvexPolyhedronManifoldGenerator::new(self.flip);
            let aabb = b.aabb(&(m1.inverse() * m2));

            for i in multiball.potential_balls_intersecting_aabb(&aabb) {
                let ball_transform = m1 * Translation::from(multiball.centers()[i].coords);
                generator.do_generate_with_exact_shapes(
                    &ball_transform,
                    &Ball::new(multiball.radius()),
                    Some(&(proc1, &multiball.contact_preprocessor(i))),
                    m2,
                    pq2,
                    cp2,
                    proc2,
                    prediction,
                    manifold);
            }

            true
        } else {
            false
        }
    }
}

impl<N: RealField> ContactManifoldGenerator<N> for MultiballConvexPolyhedronManifoldGenerator<N> {
    fn generate_contacts(
        &mut self,
        _: &dyn ContactDispatcher<N>,
        m1: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool
    {
        if !self.flip {
            self.do_generate(m1, a, proc1, m2, b, proc2, prediction, manifold)
        } else {
            self.do_generate(m2, b, proc2, m1, a, proc1, prediction, manifold)
        }
    }

    fn init_manifold(&self) -> ContactManifold<N> {
        let mut res = ContactManifold::new();
        res.set_tracking_mode(ContactTrackingMode::FeatureBased);
        res
    }
}
