use std::num::Zero;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector::Vec;
use narrow::collision_detector::CollisionDetector;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct PlaneImplicit<N, V, M, G> {
    priv margin:     N,
    priv prediction: N,
    priv contact: Option<Contact<N, V>>
}

impl<N, V, M, G> PlaneImplicit<N, V, M, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(margin: N, prediction: N) -> PlaneImplicit<N, V, M, G> {
        PlaneImplicit {
            margin:     margin,
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N: Num + NumCast + Ord + Clone,
     V: Vec<N> + Clone,
     M: Rotate<V> + Translation<V>,
     G: Implicit<V, M>>
CollisionDetector<N, V, M, Plane<N, V>, G> for PlaneImplicit<N, V, M, G> {
    #[inline]
    fn update(&mut self, ma: &M, plane: &Plane<N, V>, mb: &M, b: &G) {
        self.contact = collide_plane_implicit_shape(
            ma,
            plane,
            mb,
            b,
            &self.margin,
            &self.prediction)
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitPlane<N, V, M, G> {
    priv margin:     N,
    priv prediction: N,
    priv contact:    Option<Contact<N, V>>
}

impl<N, V, M, G> ImplicitPlane<N, V, M, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(margin: N, prediction: N) -> ImplicitPlane<N, V, M, G> {
        ImplicitPlane {
            margin:     margin,
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N: Num + NumCast + Ord + Clone,
     V: Vec<N> + Clone,
     M: Rotate<V> + Translation<V>,
     G: Implicit<V, M>>
CollisionDetector<N, V, M, G, Plane<N, V>> for ImplicitPlane<N, V, M, G> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G, mb: &M, plane: &Plane<N, V>) {
        self.contact = collide_plane_implicit_shape(mb, plane, ma, a, &self.margin, &self.prediction);
        self.contact.mutate(|mut c| { c.flip(); c });
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}

/**
 * Same as `update_collide_plane_implicit_shape` but the existing collision or
 * `None`.
 *
 * # Arguments:
 *   * `plane` - the plane to test.
 *   * `other` - the object to test against the plane.
 */
pub fn collide_plane_implicit_shape<V: Vec<N> + Clone,
                                    N: Num + NumCast + Ord + Clone,
                                    M: Rotate<V> + Translation<V>,
                                    G: Implicit<V, M>>(
                                    mplane:     &M,
                                    plane:      &Plane<N, V>,
                                    mother:     &M,
                                    other:      &G,
                                    margin:     &N,
                                    prediction: &N)
                                    -> Option<Contact<N, V>> {
    let plane_normal = mplane.rotate(&plane.normal());
    let plane_center = mplane.translation();
    let deepest;

    if *margin > Zero::zero() {
        deepest = other.support_point(mother, &-plane_normal) - plane_normal * *margin
    }
    else {
        deepest = other.support_point(mother, &-plane_normal)
    }

    let dist = plane_normal.dot(&(plane_center - deepest));

    if dist > -prediction {
        let c1 = deepest + plane_normal * dist;

        Some(Contact::new(c1, deepest, plane_normal, dist))
    }
    else {
        None
    }
}
