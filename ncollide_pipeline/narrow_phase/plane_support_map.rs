use std::any::Any;
use std::marker::PhantomData;
use na::{Translate, Rotate};
use math::{Point, Vect};
use entities::shape::Plane;
use entities::inspection;
use entities::inspection::Repr;
use queries::geometry::Contact;
use queries::geometry::contacts_internal;
use narrow_phase::{CollisionDetector, CollisionDispatcher};


/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct PlaneSupportMap<P: Point, M> {
    prediction: <P::Vect as Vect>::Scalar,
    contact:    Option<Contact<P>>,
    mat_type:   PhantomData<M> // FIXME: can we avoid this (using a generalized where clause ?)
}

impl<P: Point, M> PlaneSupportMap<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: <P::Vect as Vect>::Scalar) -> PlaneSupportMap<P, M> {
        PlaneSupportMap {
            prediction: prediction,
            contact:    None,
            mat_type:   PhantomData
        }
    }
}

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone)]
pub struct SupportMapPlane<P: Point, M> {
    prediction: <P::Vect as Vect>::Scalar,
    contact:    Option<Contact<P>>,
    mat_type:   PhantomData<M> // FIXME: can we avoid this (using a generalized where clause ?)
}

impl<P: Point, M> SupportMapPlane<P, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: <P::Vect as Vect>::Scalar) -> SupportMapPlane<P, M> {
        SupportMapPlane {
            prediction: prediction,
            contact:    None,
            mat_type:   PhantomData
        }
    }
}

impl<P, M> CollisionDetector<P, M> for PlaneSupportMap<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self,
              _:     &CollisionDispatcher<P, M>, 
              ma:    &M,
              plane: &Repr<P, M>,
              mb:    &M,
              b:     &Repr<P, M>)
              -> bool {
        let rp = plane.repr();

        if let (Some(p), Some(sm)) =
            (rp.downcast_ref::<Plane<P::Vect>>(), inspection::maybe_as_support_map::<P, M, _>(b)) {
                self.contact = contacts_internal::plane_against_support_map(ma, p, mb, sm, self.prediction);

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

impl<P, M> CollisionDetector<P, M> for SupportMapPlane<P, M>
    where P: Point,
          M: Translate<P> + Rotate<P::Vect> + Any {
    #[inline]
    fn update(&mut self,
              _:     &CollisionDispatcher<P, M>, 
              ma:    &M,
              a:     &Repr<P, M>,
              mb:    &M,
              plane: &Repr<P, M>)
              -> bool {
        let rp = plane.repr();

        if let (Some(sm), Some(p)) =
            (inspection::maybe_as_support_map::<P, M, _>(a), rp.downcast_ref::<Plane<P::Vect>>()) {
                self.contact = contacts_internal::support_map_against_plane(ma, sm, mb, p, self.prediction);

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
