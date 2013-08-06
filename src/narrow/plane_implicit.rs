use std::num::Zero;
use nalgebra::traits::dot::{Dot};
use nalgebra::traits::vector_space::VectorSpace;
use nalgebra::traits::ring::Ring;
use nalgebra::traits::scalar_op::ScalarMul;
use narrow::collision_detector::CollisionDetector;
use geom::plane::Plane;
use geom::implicit::Implicit;
use contact::Contact;

/// Collision detector between a plane and a geometry implementing the `Implicit` trait.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct PlaneImplicit<N, V, G> {
    priv contact: Option<Contact<N, V>>
}

/// Collision detector between a geometry implementing the `Implicit` trait and a plane.
/// This detector generates only one contact point. For a full manifold generation, see
/// `IncrementalContactManifoldGenerator`.
pub struct ImplicitPlane<N, V, G> {
    priv contact: Option<Contact<N, V>>
}

impl<N, V, G> PlaneImplicit<N, V, G> {
    /// Creates a new persistant collision detector between a plane and a geometry with a support
    /// mapping function.
    #[inline]
    pub fn new() -> PlaneImplicit<N, V, G> {
        PlaneImplicit { contact: None }
    }
}

impl<V: VectorSpace<N> + Dot<N> + Clone,
     N: Ring + Ord + Clone,
     G: Implicit<V>>
CollisionDetector<N, V, G, Plane<V>> for PlaneImplicit<N, V, G> {
    fn update(&mut self, b: &G, a: &Plane<V>) {
        self.contact = collide_plane_implicit_shape(a, b)
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&mut self, out_colls: &mut ~[Contact<N, V>]) {
        match self.contact {
            Some(ref c) => out_colls.push(c.clone()),
            None        => ()
        }
    }
}

impl<N, V, G> ImplicitPlane<N, V, G> {
    /// Creates a new persistant collision detector between a geometry with a support mapping
    /// function, and a plane.
    #[inline]
    pub fn new() -> ImplicitPlane<N, V, G> {
        ImplicitPlane { contact: None }
    }
}

impl<V: VectorSpace<N> + Dot<N> + Clone,
     N: Ring + Ord + Clone,
     G: Implicit<V>>
CollisionDetector<N, V, G, Plane<V>> for ImplicitPlane<N, V, G> {
    fn update(&mut self, a: &G, b: &Plane<V>) {
        self.contact = collide_plane_implicit_shape(b, a);
    }

    #[inline]
    fn num_coll(&self) -> uint {
        match self.contact {
            None    => 0,
            Some(_) => 1
        }
    }

    #[inline]
    fn colls(&mut self, out_colls: &mut ~[Contact<N, V>]) {
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
                                    G: Implicit<V>>(
                                    plane: &Plane<V>,
                                    other: &G)
                                    -> Option<Contact<N, V>> {
    let deepest = other.support_point(&-plane.normal());
    let dist    = plane.normal().dot(&(plane.center() - deepest));

    if dist > Zero::zero() {
        let c1 = deepest + plane.normal().scalar_mul(&dist);

        Some(Contact::new(deepest, c1, -plane.normal(), dist))
    }
    else {
        None
    }
}
