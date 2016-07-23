use std::marker::PhantomData;
use math::{Point, Vector, Isometry};
use geometry::shape::{Shape, Plane};
use geometry::query::Contact;
use geometry::query::contacts_internal;
use narrow_phase::{ContactGenerator, ContactDispatcher};


/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates a polyhedral approximation of the contact area contour.
#[derive(Clone)]
pub struct PlaneSupportMapContactManifoldGenerator<P: Point, M> {
    contacts:              Vec<Contact<P>>,
    manifold_eps:          <P::Vect as Vector>::Scalar,
    manifold_approx_count: usize,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapContactManifoldGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(manifold_eps:          <P::Vect as Vector>::Scalar,
               manifold_approx_count: usize)
               -> PlaneSupportMapContactManifoldGenerator<P, M> {
        PlaneSupportMapContactManifoldGenerator {
            contacts:              Vec::new(),
            manifold_eps:          manifold_eps,
            manifold_approx_count: manifold_approx_count,
            mat_type: PhantomData
        }
    }
}

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates the contour of the contact area.
#[derive(Clone)]
pub struct SupportMapPlaneContactManifoldGenerator<P: Point, M> {
    contacts:              Vec<Contact<P>>,
    manifold_eps:          <P::Vect as Vector>::Scalar,
    manifold_approx_count: usize,
    mat_type: PhantomData<M> // FIXME: can we avoid this.
}

impl<P: Point, M> SupportMapPlaneContactManifoldGenerator<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(manifold_eps:          <P::Vect as Vector>::Scalar,
               manifold_approx_count: usize)
               -> SupportMapPlaneContactManifoldGenerator<P, M> {
        SupportMapPlaneContactManifoldGenerator {
            contacts:              Vec::new(),
            manifold_eps:          manifold_eps,
            manifold_approx_count: manifold_approx_count,
            mat_type: PhantomData
        }
    }
}

impl<P, M> ContactGenerator<P, M> for PlaneSupportMapContactManifoldGenerator<P, M>
    where P: Point,
          M: Isometry<P> {
    #[inline]
    fn update(&mut self,
              _:          &ContactDispatcher<P, M>,
              ma:         &M,
              plane:      &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let (Some(p), Some(sm)) = (plane.as_shape::<Plane<P::Vect>>(), b.as_support_map()) {
            self.contacts.clear();
            let _ = contacts_internal::plane_against_support_map_manifold(ma, p, mb, sm, prediction,
                                                                          self.manifold_eps,
                                                                          self.manifold_approx_count,
                                                                          &mut self.contacts);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.contacts.len()
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        out_contacts.extend_from_slice(&self.contacts[..]);
    }
}

impl<P, M> ContactGenerator<P, M> for SupportMapPlaneContactManifoldGenerator<P, M>
    where P: Point,
          M: Isometry<P> {
    #[inline]
    fn update(&mut self,
              _:          &ContactDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              plane:      &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        if let (Some(sm), Some(p)) = (a.as_support_map(), plane.as_shape::<Plane<P::Vect>>()) {
            self.contacts.clear();
            let _ = contacts_internal::support_map_against_plane_manifold(ma, sm, mb, p, prediction,
                                                                          self.manifold_eps,
                                                                          self.manifold_approx_count,
                                                                          &mut self.contacts);

            true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_contacts(&self) -> usize {
        self.contacts.len()
    }

    #[inline]
    fn contacts(&self, out_contacts: &mut Vec<Contact<P>>) {
        out_contacts.extend_from_slice(&self.contacts[..]);
    }
}
