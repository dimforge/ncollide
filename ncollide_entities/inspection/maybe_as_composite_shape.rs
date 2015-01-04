use std::mem;
use std::intrinsics::TypeId;
use math::{Scalar, Point, Vect, Isometry};
use shape::{Compound, TriMesh, Polyline, CompositeShape};
use inspection::{Repr, ReprDesc};

/// Gets the id associated with the `CompositeShape` trait.
pub fn composite_shape_repr_id<N, P, V, M>() -> TypeId {
    TypeId::of::<&CompositeShape<N, P, V, M>>()
}

/// Converts a shape to a composite shape if possible.
#[inline]
pub fn maybe_repr_desc_as_composite_shape<'a, N, P, V, M>(desc: ReprDesc<'a>) -> Option<&'a (CompositeShape<N, P, V, M> + 'a)> {
    if desc.repr_id() == composite_shape_repr_id::<N, P, V, M>() {
        Some(unsafe { mem::transmute(desc.repr()) })
    }
    else {
        None
    }
}

/// Converts a shape to a composite shape if possible.
#[inline]
pub fn maybe_as_composite_shape<N, P, V, M, Sized? G>(g: &G) -> Option<&CompositeShape<N, P, V, M>>
    where G: Repr<N, P, V, M> {
    maybe_repr_desc_as_composite_shape(g.repr())
}

macro_rules! impl_composite_shape_repr(
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
                            TypeId::of::<&CompositeShape<N, P, V, M>>(),
                            mem::transmute(self as &CompositeShape<N, P, V, M>)
                        )
                    }
                }
            }
    }
);

impl_composite_shape_repr!(Compound<N, P, V, M>);
impl_composite_shape_repr!(TriMesh<N, P, V>);
impl_composite_shape_repr!(Polyline<N, P, V>);
