use na::Real;
use math::{Isometry, Point};
use utils::IdAllocator;
use bounding_volume::PolyhedralCone;
use shape::{Ball, FeatureId, Shape};
use query::{ContactKinematic, ContactManifold, ContactPrediction};
use query::contacts_internal;
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};

/// Collision detector between two balls.
pub struct BallBallManifoldGenerator<N: Real> {
    manifold: ContactManifold<N>,
}

impl<N: Real> Clone for BallBallManifoldGenerator<N> {
    fn clone(&self) -> BallBallManifoldGenerator<N> {
        BallBallManifoldGenerator {
            manifold: self.manifold.clone(),
        }
    }
}

impl<N: Real> BallBallManifoldGenerator<N> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new() -> BallBallManifoldGenerator<N> {
        BallBallManifoldGenerator {
            manifold: ContactManifold::new(),
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for BallBallManifoldGenerator<N> {
    fn update(
        &mut self,
        _: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<N>>(), b.as_shape::<Ball<N>>()) {
            self.manifold.set_subshape_id1(ida);
            self.manifold.set_subshape_id2(idb);
            self.manifold.save_cache_and_clear(id_alloc);

            let center_a = Point::from_coordinates(ma.translation.vector);
            let center_b = Point::from_coordinates(mb.translation.vector);
            if let Some(contact) =
                contacts_internal::ball_against_ball(&center_a, a, &center_b, b, prediction.linear)
            {
                let mut kinematic = ContactKinematic::new();
                kinematic.set_point1(FeatureId::Face(0), Point::origin(), PolyhedralCone::Full);
                kinematic.set_point2(FeatureId::Face(0), Point::origin(), PolyhedralCone::Full);
                kinematic.set_dilation1(a.radius());
                kinematic.set_dilation2(b.radius());

                let _ = self.manifold.push(contact, kinematic, id_alloc);
            }

            true
        } else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<N>>) {
        if self.manifold.len() != 0 {
            out.push(&self.manifold)
        }
    }
}
