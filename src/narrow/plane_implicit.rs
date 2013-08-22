use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::translation::Translation;
use nalgebra::traits::vector::{Vec, AlgebraicVec};
use narrow::collision_detector::CollisionDetector;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;
use ray::ray::{Ray, RayCast};

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct PlaneImplicit<N, V, M, G> {
    priv prediction: N,
    priv contact: Option<Contact<N, V>>
}

impl<N, V, M, G> PlaneImplicit<N, V, M, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> PlaneImplicit<N, V, M, G> {
        PlaneImplicit {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N: Algebraic + Num + NumCast + Ord + Clone,
     V: AlgebraicVec<N> + Clone + ToStr,
     M: Rotate<V> + Translation<V>,
     G: Implicit<N, V, M>>
CollisionDetector<N, V, M, Plane<N, V>, G> for PlaneImplicit<N, V, M, G> {
    #[inline]
    fn update(&mut self, ma: &M, plane: &Plane<N, V>, mb: &M, b: &G) {
        self.contact = collide(
            ma,
            plane,
            mb,
            b,
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

    #[inline]
    fn toi(ma: &M, dir: &V, _: &N, plane: &Plane<N, V>, mb: &M, b: &G) -> Option<N> {
        toi(ma, plane, mb, &-dir, b)
    }
}

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitPlane<N, V, M, G> {
    priv prediction: N,
    priv contact:    Option<Contact<N, V>>
}

impl<N, V, M, G> ImplicitPlane<N, V, M, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(prediction: N) -> ImplicitPlane<N, V, M, G> {
        ImplicitPlane {
            prediction: prediction,
            contact:    None
        }
    }
}

impl<N: Algebraic + Num + NumCast + Ord + Clone,
     V: AlgebraicVec<N> + Clone + ToStr,
     M: Rotate<V> + Translation<V>,
     G: Implicit<N, V, M>>
CollisionDetector<N, V, M, G, Plane<N, V>> for ImplicitPlane<N, V, M, G> {
    #[inline]
    fn update(&mut self, ma: &M, a: &G, mb: &M, plane: &Plane<N, V>) {
        self.contact = collide(mb, plane, ma, a, &self.prediction);
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

    #[inline]
    fn toi(ma: &M, dir: &V, _: &N, a: &G, mb: &M, plane: &Plane<N, V>) -> Option<N> {
        toi(mb, plane, ma, dir, a)
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
pub fn collide<V: AlgebraicVec<N> + Clone,
               N: Algebraic + Num + NumCast + Ord + Clone,
               M: Rotate<V> + Translation<V>,
               G: Implicit<N, V, M>>(
               mplane:     &M,
               plane:      &Plane<N, V>,
               mother:     &M,
               other:      &G,
               prediction: &N)
               -> Option<Contact<N, V>> {
    let plane_normal = mplane.rotate(&plane.normal());
    let plane_center = mplane.translation();
    let deepest      = other.support_point(mother, &-plane_normal);

    let dist = plane_normal.dot(&(plane_center - deepest));

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
///     * `mplane` - the plane transform.
///     * `plane`  - the plane.
///     * `mother` - the geometry transform.
///     * `dir`    - the direction of the other geometry movement.
///     * `other`  - the other geometry.
pub fn toi<N: Algebraic + Ord + Num,
           V: AlgebraicVec<N> + Clone + ToStr,
           M: Translation<V> + Rotate<V>,
           G: Implicit<N, V, M>>(
           mplane: &M,
           plane:  &Plane<N, V>,
           mother: &M,
           dir:    &V,
           other:  &G)
           -> Option<N> {
    let plane_normal  = mplane.rotate(&plane.normal());
    let closest_point = other.support_point(mother, &-plane_normal);

    plane.toi_with_ray(mplane, &Ray::new(closest_point, dir.clone()))
}
