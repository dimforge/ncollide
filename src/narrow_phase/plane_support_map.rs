use na::{Translate, Rotate, Transform};
use narrow_phase::CollisionDetector;
use geometry::Contact;
use geometry::contacts_internal;
use shape::Plane;
use support_map::SupportMap;
use ray::{Ray, RayCast};
use math::{Scalar, Point, Vect};


/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct PlaneSupportMap<N, P, V, M, G> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N: Clone, P: Clone, V: Clone, M, G> Clone for PlaneSupportMap<N, P, V, M, G> {
    fn clone(&self) -> PlaneSupportMap<N, P, V, M, G> {
        PlaneSupportMap {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<N, P, V, M, G> PlaneSupportMap<N, P, V, M, G> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> PlaneSupportMap<N, P, V, M, G> {
        PlaneSupportMap {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N, P, V, M, G> CollisionDetector<N, P, V, M, Plane<V>, G> for PlaneSupportMap<N, P, V, M, G>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
    #[inline]
    fn update(&mut self, ma: &M, plane: &Plane<V>, mb: &M, b: &G) {
        self.contact = contacts_internal::plane_against_support_map(
            ma,
            plane,
            mb,
            b,
            self.prediction)
    }

    #[inline]
    fn num_colls(&self) -> uint {
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

/// Collision detector between a plane and a shape implementing the `SupportMap` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct SupportMapPlane<N, P, V, M, G> {
    prediction: N,
    contact:    Option<Contact<N, P, V>>
}

impl<N: Clone, P: Clone, V: Clone, M, G> Clone for SupportMapPlane<N, P, V, M, G> {
    fn clone(&self) -> SupportMapPlane<N, P, V, M, G> {
        SupportMapPlane {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<N, P, V, M, G> SupportMapPlane<N, P, V, M, G> {
    /// Creates a new persistent collision detector between a plane and a shape with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> SupportMapPlane<N, P, V, M, G> {
        SupportMapPlane {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N, P, V, M, G> CollisionDetector<N, P, V, M, G, Plane<V>> for SupportMapPlane<N, P, V, M, G>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Translate<P> + Rotate<V>,
          G: SupportMap<P, V, M> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G, mb: &M, plane: &Plane<V>) {
        self.contact = contacts_internal::support_map_against_plane(ma, a, mb, plane, self.prediction);
    }

    #[inline]
    fn num_colls(&self) -> uint {
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

/// Computes the Time Of Impact of a shape and a plane.
///
/// Arguments:
/// * `mplane` - the plane transform.
/// * `plane`  - the plane.
/// * `mother` - the shape transform.
/// * `dir`    - the direction of the other shape movement.
/// * `other`  - the other shape.
pub fn toi<N, P, V, M, G>(mplane: &M, plane: &Plane<V>, mother: &M, dir: &V, other: &G) -> Option<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Rotate<V> + Transform<P>,
          G: SupportMap<P, V, M> {
    let plane_normal  = mplane.rotate(plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_transform_and_ray(mplane, &Ray::new(closest_point, dir.clone()), true)
}
