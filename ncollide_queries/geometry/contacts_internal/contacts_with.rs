//! Trait used to implement the contact determination functions.

use std::sync::Arc;
use na::{Translate, Rotation, Cross};
use na;
use entities::shape::{Ball, Plane, Cuboid, Capsule, Cone, Cylinder, Convex, Compound, Mesh, Segment,
                      Triangle};
use geometry::traits::Shape;
use geometry::Contact;
use geometry::contacts_internal;
use math::{Scalar, Point, Vect, Isometry};


/// Trait implemented by object that can be tested for contacts with another one.
pub trait ContactsWith<N, P, V, M, Sized? G> for Sized? {
    /// Computes the contact point with the smallest separating distance, or deepest penetration depth.
    fn contact(m1: &M, g1: &Self, m2: &M, g2: &G, prediction: N) -> Option<Contact<N, P, V>>;
    /// Computes all contact points, including approximations of the full contact manifold.
    fn contacts(m1: &M, g1: &Self, m2: &M, g2: &G, prediction: N, out: &mut Vec<Contact<N, P, V>>);
}

/// Computes the contact point with the smallest separating distance, or deepest penetration depth.
pub fn contact<N, P, V, M, Sized? G1, Sized? G2>(
               m1: &M, g1: &G1,
               m2: &M, g2: &G2,
               prediction: N)
               -> Option<Contact<N, P, V>>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V>,
          G1: ContactsWith<N, P, V, M, G2> {
    ContactsWith::contact(m1, g1, m2, g2, prediction)
}

/// Computes all contact points, including approximations of the full contact manifold.
pub fn contacts<N, P, V, M, Sized? G1, Sized? G2>(
                m1: &M, g1: &G1,
                m2: &M, g2: &G2,
                prediction: N,
                out: &mut Vec<Contact<N, P, V>>)
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P>,
          M:  Isometry<N, P, V>,
          G1: ContactsWith<N, P, V, M, G2> {
    ContactsWith::contacts(m1, g1, m2, g2, prediction, out)
}

/*
 *
 *
 * Impls follow.
 *
 *
 */
macro_rules! impl_contacts_with(
    ($name: ident | $g1: ty, $g2: ty) => {
        impl<N, P, V, M> ContactsWith<N, P, V, M, $g2> for $g1
            where N: Scalar,
                  P: Point<N, V>,
                  V: Vect<N> + Translate<P>,
                  M: Isometry<N, P, V> {
            #[inline]
            fn contact(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N) -> Option<Contact<N, P, V>> {
                contacts_internal::$name(m1, g1, m2, g2, prediction)
            }

            #[inline]
            fn contacts(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N, out: &mut Vec<Contact<N, P, V>>) {
                match ContactsWith::contact(m1, g1, m2, g2, prediction) {
                    Some(c) => out.push(c),
                    None    => { }
                }
            }
        }
    };
    ($name: ident, $name_manifold: ident | $g1: ty, $g2: ty) => {
        impl<N, P, V, AV, M> ContactsWith<N, P, V, M, $g2> for $g1
            where N:  Scalar,
                  P:  Point<N, V>,
                  V:  Vect<N> + Translate<P>  + Cross<AV>,
                  AV: Vect<N>,
                  M:  Isometry<N, P, V> + Rotation<AV> {
            #[inline]
            fn contact(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N) -> Option<Contact<N, P, V>> {
                contacts_internal::$name(m1, g1, m2, g2, prediction)
            }

            #[inline]
            fn contacts(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N, out: &mut Vec<Contact<N, P, V>>) {
                contacts_internal::$name_manifold(m1, g1, m2, g2, prediction, out)
            }
        }
    };
    ($name: ident, $unused0: ident, $unused1: ident | $g1: ty, $g2: ty) => {
        impl<N, P, V, AV, M> ContactsWith<N, P, V, M, $g2> for $g1
            where N:  Scalar,
                  P:  Point<N, V>,
                  V:  Vect<N> + Translate<P>  + Cross<AV>,
                  AV: Vect<N>,
                  M:  Isometry<N, P, V> + Rotation<AV> {
            #[inline]
            fn contact(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N) -> Option<Contact<N, P, V>> {
                contacts_internal::$name(m1, g1, m2, g2, prediction)
            }

            #[inline]
            fn contacts(m1: &M, g1: &$g1, m2: &M, g2: &$g2, prediction: N, out: &mut Vec<Contact<N, P, V>>) {
                contacts_internal::generate_contact_manifold(
                    m1, g1, m2, g2, prediction, contacts_internal::$name, out
                )
            }
        }
    }
);

apply_with_mixed_args!(impl_contacts_with,
                       plane_against_support_map,       default, default |
                       support_map_against_plane,       default, default |
                       support_map_against_support_map, default, default |
                       concave_shape_against_shape,     manifold_concave_shape_against_shape |
                       shape_against_concave_shape,     manifold_shape_against_concave_shape);

impl<N, P, V, M> ContactsWith<N, P, V, M, Ball<N>> for Ball<N>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Isometry<N, P, V> {
    #[inline]
    fn contact(m1: &M, g1: &Ball<N>,
               m2: &M, g2: &Ball<N>,
               prediction: N)
               -> Option<Contact<N, P, V>> {
        let p1 = m1.translate(&na::orig());
        let p2 = m2.translate(&na::orig());
        contacts_internal::ball_against_ball(&p1, g1, &p2, g2, prediction)
    }

    #[inline]
    fn contacts(m1: &M, g1: &Ball<N>,
                m2: &M, g2: &Ball<N>,
                prediction: N,
                out: &mut Vec<Contact<N, P, V>>) {
        match ContactsWith::contact(m1, g1, m2, g2, prediction) {
            Some(c) => out.push(c),
            None    => { }
        }
    }
}

/* FIXME: DST: ICE
impl<N, P, V, M> ContactsWith<N, P, V, M, Shape<N, P, V, M> + Send + Sync> for Shape<N, P, V, M> + Send + Sync
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Isometry<N, P, V> {
    #[inline]
    fn contact(m1: &M, g1: &Shape<N, P, V, M> + Send + Sync,
               m2: &M, g2: &Shape<N, P, V, M> + Send + Sync,
               prediction: N)
               -> Option<Contact<N, P, V>> {
        unimplemented!()
        // contacts_internal::$name(m1, g1, m2, g2, prediction)
    }

    #[inline]
    fn contacts(m1: &M, g1: &Shape<N, P, V, M> + Send + Sync,
                m2: &M, g2: &Shape<N, P, V, M> + Send + Sync,
                prediction: N,
                out: &mut Vec<Contact<N, P, V>>) {
        match ContactsWith::contact(m1, g1, m2, g2, prediction) {
            Some(c) => out.push(c),
            None    => { }
        }
    }
}
*/
