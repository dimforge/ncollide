use std::marker::PhantomData;
use math::{Isometry, Point};
use utils::IdAllocator;
use geometry::shape::{Plane, Shape};
use geometry::query::{Contact, ContactManifold, ContactPrediction};
use geometry::query::contacts_internal;
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
#[derive(Clone)]
pub struct PlaneSupportMapContactGenerator<P: Point, M> {
    manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapContactGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapContactGenerator<P, M> {
        PlaneSupportMapContactGenerator {
            manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }
}

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapPlaneContactGenerator<P: Point, M> {
    manifold: ContactManifold<P>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this.
}

impl<P: Point, M> SupportMapPlaneContactGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneContactGenerator<P, M> {
        SupportMapPlaneContactGenerator {
            manifold: ContactManifold::new(),
            mat_type: PhantomData,
        }
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M> for PlaneSupportMapContactGenerator<P, M> {
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        ma: &M,
        plane: &Shape<P, M>,
        mb: &M,
        b: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(p), Some(sm)) = (plane.as_shape::<Plane<P::Vector>>(), b.as_support_map()) {
            self.manifold.save_cache_and_clear();
            let contact =
                contacts_internal::plane_against_support_map(ma, p, mb, sm, prediction.linear);
            unimplemented!();
            // self.manifold.push(contact)

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
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        out.push(&self.manifold)
    }
}

impl<P: Point, M: Isometry<P>> ContactGenerator<P, M> for SupportMapPlaneContactGenerator<P, M> {
    #[inline]
    fn update(
        &mut self,
        _: &ContactDispatcher<P, M>,
        ma: &M,
        a: &Shape<P, M>,
        mb: &M,
        plane: &Shape<P, M>,
        prediction: &ContactPrediction<P::Real>,
        id_alloc: &mut IdAllocator,
    ) -> bool {
        if let (Some(sm), Some(p)) = (a.as_support_map(), plane.as_shape::<Plane<P::Vector>>()) {
            let contact =
                contacts_internal::support_map_against_plane(ma, sm, mb, p, prediction.linear);
            unimplemented!();

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
    fn contacts<'a: 'b, 'b>(&'a self, out: &'b mut Vec<&'a ContactManifold<P>>) {
        if self.manifold.len() != 0 {
            out.push(&self.manifold)
        }
    }
}
