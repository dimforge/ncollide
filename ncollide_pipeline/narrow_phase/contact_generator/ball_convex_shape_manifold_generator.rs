use std::marker::PhantomData;

use alga::linear::Translation;
use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::bounding_volume::PolyhedralCone;
use geometry::shape::{Ball, FeatureId, Shape};
use geometry::query::{ContactKinematic, ContactManifold, ContactPrediction};
use geometry::query::contacts_internal;
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Collision detector between two balls.
pub struct BallConvexShapeManifoldGenerator<P: Point, M> {
    flip: bool,
    manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallConvexShapeManifoldGenerator<P, M> {
    fn clone(&self) -> BallConvexShapeManifoldGenerator<P, M> {
        BallConvexShapeManifoldGenerator {
            flip: self.flip,
            manifold: self.manifold.clone(),
            mat_type: PhantomData,
        }
    }
}

impl<P: Point, M: Isometry<P>> BallConvexShapeManifoldGenerator<P, M> {
    /// Creates a new persistent collision detector between two balls.
    #[inline]
    pub fn new(flip: bool) -> BallConvexShapeManifoldGenerator<P, M> {
        BallConvexShapeManifoldGenerator {
            flip,
            manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }

    fn do_update(
        &mut self,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
        flip: bool,
    ) -> bool {
        if let (Some(a), Some(pq)) = (a.as_shape::<Ball<P::Real>>(), b.as_point_query()) {
            self.manifold.save_cache_and_clear(id_alloc);

            let center_a = P::from_coordinates(ma.translation().to_vector());
            let proj = pq.project_point(mb, &center_a, false);

            true
        } else {
            false
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M> for BallConvexShapeManifoldGenerator<P, M> {
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if !self.flip {
            self.do_update(ma, a, mb, b, prediction, id_alloc, false)
        } else {
            self.do_update(mb, b, ma, a, prediction, id_alloc, true)
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        if self.manifold.len() != 0 {
            out.push(&self.manifold)
        }
    }
}
