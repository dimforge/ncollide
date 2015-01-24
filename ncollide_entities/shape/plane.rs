//! Support mapping based Plane shape.
use std::any::Any;
use std::mem;
use std::any::TypeId;
use na;
use na::Norm;
use inspection::{Repr, ReprDesc};
use math::Scalar;

/// SupportMap description of a plane.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Plane<V> {
    /// The plane normal.
    normal: V
}

#[old_impl_check]
impl<N: Scalar, V: Norm<N>> Plane<V> {
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

impl<N, P, V, M> Repr<N, P, V, M> for Plane<V>
    where V: Send + Sync {
    #[inline(always)]
    fn repr(&self) -> ReprDesc {
        unsafe {
            ReprDesc::new(
                TypeId::of::<Plane<V>>(),
                TypeId::of::<&Any>(),
                mem::transmute(self as &Any)
            )
        }
    }
}
