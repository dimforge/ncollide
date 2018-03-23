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
    contact_manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> Clone for BallConvexShapeManifoldGenerator<P, M> {
    fn clone(&self) -> BallConvexShapeManifoldGenerator<P, M> {
        BallConvexShapeManifoldGenerator {
            flip: self.flip,
            contact_manifold: self.contact_manifold.clone(),
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
            contact_manifold: ContactManifold::new(),
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
            self.contact_manifold.save_cache_and_clear(id_alloc);

            let center_a = P::from_coordinates(ma.translation().to_vector());
            let (proj, feature) = pq.project_point_with_feature(mb, &center_a);

            {
                let kinematic =
                    Self::contact_kinematic(m1, &self.manifold1, f1, m2, &self.manifold2, f2);
                let local1 = m1.inverse_transform_point(&c.world1);
                let local2 = m2.inverse_transform_point(&c.world2);
                let n1 = g1.normal_cone(f1);
                let n2 = g2.normal_cone(f2);

                self.contact_manifold
                    .push(c, local1, local2, n1, n2, f1, f2, kinematic, ids);
            }

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
        self.contact_manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        if self.contact_manifold.len() != 0 {
            out.push(&self.contact_manifold)
        }
    }
}
