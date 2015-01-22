use std::mem;
use std::any::TypeId;
use math::{Scalar, Point, Vect, Isometry};
use shape::{Ball, Capsule, Cone, Convex, Cuboid, Cylinder, Segment, Triangle};
use support_map::SupportMap;
use inspection::{Repr, ReprDesc};

/// Gets the id associated with the `SupportMap` trait.
pub fn support_map_repr_id<P, V, M>() -> TypeId {
    TypeId::of::<&SupportMap<P, V, M>>()
}

/// Converts a shape descriptor to a support map if possible.
#[inline]
pub fn maybe_repr_desc_as_support_map<'a, P, V, M>(desc: ReprDesc<'a>) -> Option<&'a (SupportMap<P, V, M> + 'a)> {
    if desc.repr_id() == support_map_repr_id::<P, V, M>() {
        Some(unsafe { mem::transmute(desc.repr()) })
    }
    else {
        None
    }
}

/// Converts a shape to a support map if possible.
#[inline]
pub fn maybe_as_support_map<N, P, V, M, G: ?Sized>(g: &G) -> Option<&SupportMap<P, V, M>>
    where G: Repr<N, P, V, M> {
    maybe_repr_desc_as_support_map(g.repr())
}

macro_rules! impl_support_map_repr(
    ($t: ty) => {
        impl<N, P, V, M> Repr<N, P, V, M> for $t
            where N: Scalar,
                  P: Point<N, V>,
                  V: Vect<N>,
                  M: Isometry<N, P, V> {
                #[inline(always)]
                fn repr(&self) -> ReprDesc {
                    unsafe {
                        ReprDesc::new(
                            TypeId::of::<$t>(),
                            TypeId::of::<&SupportMap<P, V, M>>(),
                            mem::transmute(self as &SupportMap<P, V, M>)
                        )
                    }
                }
            }
    }
);

impl_support_map_repr!(Ball<N>);
impl_support_map_repr!(Capsule<N>);
impl_support_map_repr!(Cone<N>);
impl_support_map_repr!(Convex<P>);
impl_support_map_repr!(Cuboid<V>);
impl_support_map_repr!(Cylinder<N>);
impl_support_map_repr!(Segment<P>);
impl_support_map_repr!(Triangle<P>);
