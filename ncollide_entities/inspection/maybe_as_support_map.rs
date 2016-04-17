use std::mem;
use std::any::{Any, TypeId};
use math::{Point, Vector, Isometry};
use shape::{Ball, Capsule, Cone, ConvexHull, Cuboid, Cylinder, Segment, Triangle};
use support_map::SupportMap;
use inspection::{Repr, ReprDesc};

/// Gets the id associated with the `SupportMap` trait.
pub fn support_map_repr_id<P: Any, M: Any>() -> TypeId {
    TypeId::of::<&SupportMap<P, M>>()
}

/// Converts a shape descriptor to a support map if possible.
#[inline]
pub fn maybe_repr_desc_as_support_map<'a, P: Any, M: Any>(desc: ReprDesc<'a, P, M>) -> Option<&'a (SupportMap<P, M> + 'a)> {
    if desc.repr_id() == support_map_repr_id::<P, M>() {
        Some(unsafe { mem::transmute(desc.repr()) })
    }
    else {
        None
    }
}

/// Converts a shape to a support map if possible.
#[inline]
pub fn maybe_as_support_map<P, M, G: ?Sized>(g: &G) -> Option<&SupportMap<P, M>>
    where P: Any,
          M: Any,
          G: Repr<P, M> {
    maybe_repr_desc_as_support_map(g.repr())
}

macro_rules! impl_support_map_repr(
    ($t: ty) => {
        impl<P, M> Repr<P, M> for $t
            where P: Point,
                  M: Isometry<P, P::Vect> {
                #[inline(always)]
                fn repr(&self) -> ReprDesc<P, M> {
                    unsafe {
                        ReprDesc::new(
                            TypeId::of::<$t>(),
                            TypeId::of::<&SupportMap<P, M>>(),
                            mem::transmute(self as &SupportMap<P, M>)
                        )
                    }
                }
            }
    }
);

impl_support_map_repr!(Ball<<P::Vect as Vector>::Scalar>);
impl_support_map_repr!(Capsule<<P::Vect as Vector>::Scalar>);
impl_support_map_repr!(Cone<<P::Vect as Vector>::Scalar>);
impl_support_map_repr!(ConvexHull<P>);
impl_support_map_repr!(Cuboid<P::Vect>);
impl_support_map_repr!(Cylinder<<P::Vect as Vector>::Scalar>);
impl_support_map_repr!(Segment<P>);
impl_support_map_repr!(Triangle<P>);
