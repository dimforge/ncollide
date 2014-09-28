use na::{Translation, Rotate};
use na;
use narrow::{CollisionDetector, Contact};
use geom::Plane;
use implicit::Implicit;
use ray::{Ray, RayCast};
use math::{Scalar, Vect, Matrix};

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct PlaneImplicit<G> {
    prediction: Scalar,
    contact:    Option<Contact>
}

impl<G> Clone for PlaneImplicit<G> {
    fn clone(&self) -> PlaneImplicit<G> {
        PlaneImplicit {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<G> PlaneImplicit<G> {
    /// Creates a new persistent collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: Scalar) -> PlaneImplicit<G> {
        PlaneImplicit {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<G: Implicit<Vect, Matrix>> CollisionDetector<Plane, G> for PlaneImplicit<G> {
    #[inline]
    fn update(&mut self, ma: &Matrix, plane: &Plane, mb: &Matrix, b: &G) {
        self.contact = collide(
            ma,
            plane,
            mb,
            b,
            &self.prediction)
    }

    #[inline]
    fn num_colls(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }

    #[inline]
    fn toi(_:     Option<PlaneImplicit<G>>,
           ma:    &Matrix,
           dir:   &Vect,
           _:     &Scalar,
           plane: &Plane,
           mb:    &Matrix,
           b:     &G) -> Option<Scalar> {
        toi(ma, plane, mb, &-dir, b)
    }
}

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
///
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
#[deriving(Encodable, Decodable)]
pub struct ImplicitPlane<G> {
    prediction: Scalar,
    contact:    Option<Contact>
}

impl<G> Clone for ImplicitPlane<G> {
    fn clone(&self) -> ImplicitPlane<G> {
        ImplicitPlane {
            prediction: self.prediction.clone(),
            contact:    self.contact.clone()
        }
    }
}

impl<G> ImplicitPlane<G> {
    /// Creates a new persistent collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: Scalar) -> ImplicitPlane<G> {
        ImplicitPlane {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<G: Implicit<Vect, Matrix>> CollisionDetector<G, Plane> for ImplicitPlane<G> {
    #[inline]
    fn update(&mut self, ma: &Matrix, a: &G, mb: &Matrix, plane: &Plane) {
        self.contact = collide(mb, plane, ma, a, &self.prediction);

        match self.contact {
            Some(ref mut c) => c.flip(),
            None            => { }
        }
    }

    #[inline]
    fn num_colls(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut Vec<Contact>) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }

    #[inline]
    fn toi(_:     Option<ImplicitPlane<G>>,
           ma:    &Matrix,
           dir:   &Vect,
           _:     &Scalar,
           a:     &G,
           mb:    &Matrix,
           plane: &Plane) -> Option<Scalar> {
        toi(mb, plane, ma, dir, a)
    }
}

/// Same as `update_collide_plane_implicit_shape` but the existing collision or `None`.
///
/// # Arguments:
/// * `plane` - the plane to test.
/// * `other` - the object to test against the plane.
pub fn collide<G: Implicit<Vect, Matrix>>(
               mplane:     &Matrix,
               plane:      &Plane,
               mother:     &Matrix,
               other:      &G,
               prediction: &Scalar)
               -> Option<Contact> {
    let plane_normal = mplane.rotate(&plane.normal());
    let plane_center = mplane.translation();
    let deepest      = other.support_point(mother, &-plane_normal);

    let dist = na::dot(&plane_normal, &(plane_center - deepest));

    if dist > -prediction {
        let c1 = deepest + plane_normal * dist;

        Some(Contact::new(c1, deepest, plane_normal, dist))
    }
    else {
        None
    }
}

/// Computes the Time Of Impact of a geometry and a plane.
///
/// Arguments:
/// * `mplane` - the plane transform.
/// * `plane`  - the plane.
/// * `mother` - the geometry transform.
/// * `dir`    - the direction of the other geometry movement.
/// * `other`  - the other geometry.
pub fn toi<G: Implicit<Vect, Matrix>>(
           mplane: &Matrix,
           plane:  &Plane,
           mother: &Matrix,
           dir:    &Vect,
           other:  &G)
           -> Option<Scalar> {
    let plane_normal  = mplane.rotate(&plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_transform_and_ray(mplane, &Ray::new(closest_point, dir.clone()), true)
}
