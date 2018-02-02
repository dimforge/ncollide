use std::marker::PhantomData;
use math::{Isometry, Point};
use geometry::shape::{Plane, Shape};
use geometry::query::{Contact, ContactPrediction};
use geometry::query::contacts_internal;
use narrow_phase::{ContactDispatcher, ContactGenerator};

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct PlaneSupportMapContactGenerator<P: Point, M> {
    contact: Option<Contact<P>>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapContactGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapContactGenerator<P, M> {
        PlaneSupportMapContactGenerator {
            contact: None,
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
    contact: Option<Contact<P>>,
    mat_type: PhantomData<M>, // FIXME: can we avoid this.
}

impl<P: Point, M> SupportMapPlaneContactGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneContactGenerator<P, M> {
        SupportMapPlaneContactGenerator {
            contact: None,
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
    ) -> bool {
        if let (Some(p), Some(sm)) = (plane.as_shape::<Plane<P::Vector>>(), b.as_support_map()) {
            self.contact = contacts_internal::plane_against_support_map(ma, p, mb, sm, prediction.linear);

            true
        } else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        match self.contact {
            None => 0,
            Some(_) => 1,
        }
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_contacts.push(c.clone()),
            None => (),
        }
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
    ) -> bool {
        if let (Some(sm), Some(p)) = (a.as_support_map(), plane.as_shape::<Plane<P::Vector>>()) {
            self.contact = contacts_internal::support_map_against_plane(ma, sm, mb, p, prediction.linear);

            true
        } else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        match self.contact {
            None => 0,
            Some(_) => 1,
        }
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_contacts.push(c.clone()),
            None => (),
        }
    }
}
