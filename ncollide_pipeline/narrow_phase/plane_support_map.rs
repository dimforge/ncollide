use na::{Translate, Rotate};
use math::{Scalar, Point, Vect};
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
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub struct PlaneSupportMap<N, P, V, M> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N, P, V, M> PlaneSupportMap<N, P, V, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> PlaneSupportMap<N, P, V, M> {
        PlaneSupportMap {
            prediction: prediction,
            contact:    None
        }
    }
}

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[derive(Clone, RustcEncodable, RustcDecodable)]
pub struct SupportMapPlane<N, P, V, M> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N, P, V, M> SupportMapPlane<N, P, V, M> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> SupportMapPlane<N, P, V, M> {
        SupportMapPlane {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M>
for PlaneSupportMap<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V> {
    #[inline]
    fn update(&mut self,
              _:     &CollisionDispatcher<N, P, V, M>, 
              ma:    &M,
              plane: &Repr<N, P, V, M>,
              mb:    &M,
              b:     &Repr<N, P, V, M>)
              -> bool {
        let rp = plane.repr();

        if let (Some(p), Some(sm)) =
            (rp.downcast_ref::<Plane<V>>(), inspection::maybe_as_support_map(b)) {
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
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}

impl<N, P, V, M> CollisionDetector<N, P, V, M> for SupportMapPlane<N, P, V, M>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V> {
    #[inline]
    fn update(&mut self,
              _:     &CollisionDispatcher<N, P, V, M>, 
              ma:    &M,
              a:     &Repr<N, P, V, M>,
              mb:    &M,
              plane: &Repr<N, P, V, M>)
              -> bool {
        let rp = plane.repr();

        if let (Some(sm), Some(p)) =
            (inspection::maybe_as_support_map(a), rp.downcast_ref::<Plane<V>>()) {
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
    fn colls(&self, out_colls: &mut Vec<Contact<N, P, V>>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}
