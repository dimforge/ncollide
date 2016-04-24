use std::any::Any;
use std::marker::PhantomData;
use na::{Translate, Rotate};
use math::{Point, Vector};
use entities::shape::Plane;
use entities::inspection::Shape;
use queries::geometry::Contact;
use queries::geometry::contacts_internal;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct PlaneSupportMapCollisionDetector<P: Point, M> {
    contact:  Option<Contact<P>>,
    mat_type: PhantomData<M> // FIXME: can we avoid this?
}

impl<P: Point, M> PlaneSupportMapCollisionDetector<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneSupportMapCollisionDetector<P, M> {
        PlaneSupportMapCollisionDetector {
            contact:  None,
            mat_type: PhantomData
        }
    }
}

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapPlaneCollisionDetector<P: Point, M> {
    contact:  Option<Contact<P>>,
    mat_type: PhantomData<M> // FIXME: can we avoid this.
}

impl<P: Point, M> SupportMapPlaneCollisionDetector<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new() -> SupportMapPlaneCollisionDetector<P, M> {
        SupportMapPlaneCollisionDetector {
            contact:  None,
            mat_type: PhantomData
        }
    }
}

impl<P, M> CollisionDetector<P, M> for PlaneSupportMapCollisionDetector<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self,
              _:          &CollisionDispatcher<P, M>,
              ma:         &M,
              plane:      &Shape<P, M>,
              mb:         &M,
              b:          &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        let rp = plane.desc();
        let rb = b.desc();

        if let (Some(p), Some(sm)) = (rp.as_shape::<Plane<P::Vect>>(), rb.as_support_map()) {
                self.contact = contacts_internal::plane_against_support_map(ma, p, mb, sm, prediction);

                true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_colls(&self) -> usize {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}

impl<P, M> CollisionDetector<P, M> for SupportMapPlaneCollisionDetector<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self,
              _:          &CollisionDispatcher<P, M>,
              ma:         &M,
              a:          &Shape<P, M>,
              mb:         &M,
              plane:      &Shape<P, M>,
              prediction: <P::Vect as Vector>::Scalar)
              -> bool {
        let rp = plane.desc();
        let ra = a.desc();

        if let (Some(sm), Some(p)) = (ra.as_support_map(), rp.as_shape::<Plane<P::Vect>>()) {
                self.contact = contacts_internal::support_map_against_plane(ma, sm, mb, p, prediction);

                true
        }
        else {
            false
        }
    }

    #[inline]
    fn num_colls(&self) -> usize {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact<P>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}
