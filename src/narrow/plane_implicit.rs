use std::num::Zero;
use nalgebra::traits::dot::Dot;
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::ring::Ring;
use nalgebra::traits::scalar_op::ScalarMul;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::translation::Translation;
use narrow::collision_detector::CollisionDetector;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct PlaneImplicit<N, V, M, G> {
    priv margin:  N,
    priv contact: Option<Contact<N, V>>
}

/// Collision detector between a geometry implementing the `Implicit` trait and a plane.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitPlane<N, V, M, G> {
    priv margin:  N,
    priv contact: Option<Contact<N, V>>
}

impl<N, V, M, G> PlaneImplicit<N, V, M, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new(margin: N) -> PlaneImplicit<N, V, M, G> {
        PlaneImplicit {
            margin:  margin,
            contact: None
        }
    }
}

impl<N: Ring + Ord + Clone,
     V: VectorSpace<N> + Dot<N> + Clone,
     M: Rotate<V> + Translation<V>,
     G: Implicit<V, M>>
CollisionDetector<N, V, M, G, Plane<V>> for PlaneImplicit<N, V, M, G> {
    fn update(&mut self, mb: &M, b: &G, ma: &M, a: &Plane<V>) {
        self.contact = collide_plane_implicit_shape(ma, a, mb, b, &self.margin)
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

impl<N, V, M, G> ImplicitPlane<N, V, M, G> {
    /// Creates a new persistant collision detector between a geometry with a support mapping
    /// function, and a plane.
    #[inline]
    pub fn new(margin: N) -> ImplicitPlane<N, V, M, G> {
        ImplicitPlane {
            margin:  margin,
            contact: None
        }
    }
}

impl<V: VectorSpace<N> + Dot<N> + Clone,
     N: Ring + Ord + Clone,
     M: Rotate<V> + Translation<V>,
     G: Implicit<V, M>>
CollisionDetector<N, V, M, G, Plane<V>> for ImplicitPlane<N, V, M, G> {
    fn update(&mut self, ma: &M, a: &G, mb: &M, b: &Plane<V>) {
        self.contact = collide_plane_implicit_shape(mb, b, ma, a, &self.margin);
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
 * `None`. In the created collsion:
 *   * `world1` - designs the collision point on the object.
 *   * `world2` - designs the collision point on the object.
 *   * `normal` - designs the opposite of the plane normal.
 *
 * # Arguments:
 *   * `plane` - the plane to test.
 *   * `other` - the object to test against the plane.
 */
pub fn collide_plane_implicit_shape<V: VectorSpace<N> + Dot<N> + Clone,
                                    N: Ring + Ord + Clone,
                                    M: Rotate<V> + Translation<V>,
                                    G: Implicit<V, M>>(
                                    mplane: &M,
                                    plane:  &Plane<V>,
                                    mother: &M,
                                    other:  &G,
                                    margin: &N)
                                    -> Option<Contact<N, V>> {
    let plane_normal = mplane.rotate(&plane.normal());

    let plane_center = mplane.translation();

    let deepest;

    if *margin > Zero::zero() {
        deepest = other.support_point(mother, &-plane_normal) - plane_normal.scalar_mul(margin)
    }
    else {
        deepest = other.support_point(mother, &-plane_normal)
    }

    let dist    = plane_normal.dot(&(plane_center - deepest));

    if dist > Zero::zero() {
        let c1 = deepest + plane_normal.scalar_mul(&dist);

        Some(Contact::new(deepest, c1, -plane_normal, dist))
    }
    else {
        None
    }
}
