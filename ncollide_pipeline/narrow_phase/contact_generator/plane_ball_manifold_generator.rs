use std::marker::PhantomData;
use alga::linear::Translation;
use na::{self, Unit};

use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::shape::{ConvexPolyface, FeatureId, Plane, Shape, Ball};
use geometry::bounding_volume::PolyhedralCone;
use geometry::query::{Contact, ContactKinematic, ContactManifold, ContactPrediction};
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Collision detector between g1 plane and g1 shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneBallManifoldGenerator<P: Point, M> {
    flip: bool,
    manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M: Isometry<P>> PlaneBallManifoldGenerator<P, M> {
    /// Creates g1 new persistent collision detector between g1 plane and g1 shape with g1 support
    /// mapping function.
    #[inline]
    pub fn new(flip: bool) -> PlaneBallManifoldGenerator<P, M> {
        PlaneBallManifoldGenerator {
            flip,
            manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }

    #[inline]
    fn do_update(
        &mut self,
        m1: &M,
        g1: &Shape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
        flip: bool,
    ) -> bool {
        if let (Some(plane), Some(ball)) = (g1.as_shape::<Plane<P::Vector>>(), g2.as_shape::<Ball<P::Real>>()) {
            self.manifold.save_cache_and_clear(id_alloc);

            let plane_normal = m1.transform_unit_vector(plane.normal());
            let plane_center = P::from_coordinates(m1.translation().to_vector());

            let ball_center = P::from_coordinates(m2.translation().to_vector());
            let dist = na::dot(&(ball_center - plane_center), plane_normal.as_ref());
            let depth = -dist + ball.radius();

            if depth > -prediction.linear {
                let world1 = ball_center + *plane_normal * (-dist);
                let world2 = ball_center + *plane_normal * (-ball.radius());

                let local1 = m1.inverse_transform_point(&world1);
                let local2 = P::origin();

                let f1 = FeatureId::face(0, 0);
                let f2 = FeatureId::face(0, 0);
                let mut kinematic = ContactKinematic::new();
                let contact;

                if !flip {
                    contact = Contact::new(world1, world2, plane_normal, depth);
                    kinematic.set_plane1(f1, local1, *plane.normal());
                    kinematic.set_point2(f2, local2, PolyhedralCone::new());
                    kinematic.set_dilation2(ball.radius());
                } else {
                    contact = Contact::new(world2, world1, -plane_normal, depth);
                    kinematic.set_point1(f2, local2, PolyhedralCone::new());
                    kinematic.set_dilation1(ball.radius());                    
                    kinematic.set_plane2(f1, local1, *plane.normal());
                }

                let _ = self.manifold.push(contact, kinematic, id_alloc);
            }

            true
        } else {
            false
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M> for PlaneBallManifoldGenerator<P, M> {
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        m1: &M,
        g1: &Shape<P, M>,
        m2: &M,
        g2: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if !self.flip {
            self.do_update(m1, g1, m2, g2, prediction, id_alloc, false)
        } else {
            self.do_update(m2, g2, m1, g1, prediction, id_alloc, true)
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.manifold.len()
    }

    #[inline]
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        out.push(&self.manifold)
    }
}
