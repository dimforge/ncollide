use bounding_volume::ConicalApproximation;
use math::{Isometry, Point};
use na::Real;
use pipeline::narrow_phase::{ContactDispatcher, ContactManifoldGenerator};
use query::contacts_internal;
use query::{ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry};
use shape::{Ball, FeatureId, Shape};
use utils::IdAllocator;

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

    fn do_update_to(
        _: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        if let (Some(a), Some(b)) = (a.as_shape::<Ball<N>>(), b.as_shape::<Ball<N>>()) {
            let center_a = Point::from_coordinates(ma.translation.vector);
            let center_b = Point::from_coordinates(mb.translation.vector);
            if let Some(contact) = contacts_internal::ball_against_ball(
                &center_a,
                a,
                &center_b,
                b,
                prediction.linear(),
            ) {
                let mut kinematic = ContactKinematic::new();
                kinematic.set_approx1(
                    FeatureId::Face(0),
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_approx2(
                    FeatureId::Face(0),
                    Point::origin(),
                    NeighborhoodGeometry::Point,
                );
                kinematic.set_dilation1(a.radius());
                kinematic.set_dilation2(b.radius());

                let _ = manifold.push(contact, Point::origin(), kinematic, id_alloc);
            }

            true
        } else {
            false
        }
    }
}

impl<N: Real> ContactManifoldGenerator<N> for BallBallManifoldGenerator<N> {
    fn update(
        &mut self,
        disp: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        self.manifold.save_cache_and_clear(id_alloc);
        Self::do_update_to(
            disp,
            ida,
            ma,
            a,
            idb,
            mb,
            b,
            prediction,
            id_alloc,
            &mut self.manifold,
        )
    }

    fn update_to(
        &mut self,
        disp: &ContactDispatcher<N>,
        ida: usize,
        ma: &Isometry<N>,
        a: &Shape<N>,
        idb: usize,
        mb: &Isometry<N>,
        b: &Shape<N>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        Self::do_update_to(disp, ida, ma, a, idb, mb, b, prediction, id_alloc, manifold)
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
