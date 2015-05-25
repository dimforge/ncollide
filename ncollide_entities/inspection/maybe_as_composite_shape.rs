use std::mem;
use std::any::{TypeId, Any};
use math::{Point, Vect, Isometry};
use shape::{Compound, TriMesh, Polyline, CompositeShape};
use inspection::{Repr, ReprDesc};

/// Gets the id associated with the `CompositeShape` trait.
pub fn composite_shape_repr_id<P: Any, M: Any>() -> TypeId {
    TypeId::of::<&CompositeShape<P, M>>()
}

/// Converts a shape to a composite shape if possible.
#[inline]
pub fn maybe_repr_desc_as_composite_shape<'a, P: Any, M: Any>(desc: ReprDesc<'a>) -> Option<&'a (CompositeShape<P, M> + 'a)> {
    if desc.repr_id() == composite_shape_repr_id::<P, M>() {
        Some(unsafe { mem::transmute(desc.repr()) })
    }
    else {
        None
    }
}

/// Converts a shape to a composite shape if possible.
#[inline]
pub fn maybe_as_composite_shape<P, M, G: ?Sized>(g: &G) -> Option<&CompositeShape<P, M>>
    where P: Any,
          M: Any,
          G: Repr<P, M> {
    maybe_repr_desc_as_composite_shape(g.repr())
}

macro_rules! impl_composite_shape_repr(
    ($t: ty) => {
        impl<P, M> Repr<P, M> for $t
            where P: Point,
                  M: Isometry<P, P::Vect> {
                #[inline(always)]
                fn repr(&self) -> ReprDesc {
                    unsafe {
                        ReprDesc::new(
                            TypeId::of::<$t>(),
                            TypeId::of::<&CompositeShape<P, M>>(),
                            mem::transmute(self as &CompositeShape<P, M>)
                        )
                    }
                }
            }
    }
);

impl_composite_shape_repr!(Compound<P, M>);
impl_composite_shape_repr!(TriMesh<P>);
impl_composite_shape_repr!(Polyline<P>);
