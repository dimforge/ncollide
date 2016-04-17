//! Support mapping based Plane shape.
use std::any::Any;
use std::mem;
use std::any::TypeId;
use na;
use inspection::{Repr, ReprDesc};
use math::{Vector, Point};

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Plane<V> {
    /// The plane normal.
    normal: V
}

impl<V: Vector> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub fn new(normal: V) -> Plane<V> {
        unsafe { Plane::new_normalized(na::normalize(&normal)) }
    }
}

impl<V> Plane<V> {
    /// Builds a new plane from its center and its normal.
    #[inline]
    pub unsafe fn new_normalized(normal: V) -> Plane<V> {
        Plane {
            normal: normal
        }
    }

    /// The plane normal.
    #[inline]
    pub fn normal(&self) -> &V {
        &self.normal
    }
}

impl<P, M> Repr<P, M> for Plane<P::Vect>
    where P: Point {
    #[inline(always)]
    fn repr(&self) -> ReprDesc<P, M> {
        unsafe {
            ReprDesc::new(
                TypeId::of::<Plane<P::Vect>>(),
                TypeId::of::<&Any>(),
                mem::transmute(self as &Any)
            )
        }
    }
}
