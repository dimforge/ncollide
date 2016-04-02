use std::mem;
use std::slice;
use na::{Vec2, Vec3, Pnt2, Pnt3};

/// Trait that transforms thing to a slice of u8.
pub trait AsBytes {
    fn as_bytes<'a>(&'a self) -> &'a [u8];
}

macro_rules! generic_as_bytes_impl(
    ($t: ident, $dim: expr) => (
        impl<N: Copy> AsBytes for $t<N> {
            #[inline(always)]
            fn as_bytes<'a>(&'a self) -> &'a [u8] {
                unsafe {
                    slice::from_raw_parts(mem::transmute(self), mem::size_of::<N>() * $dim)
                }
            }
        }
    )
);

generic_as_bytes_impl!(Vec2, 2);
generic_as_bytes_impl!(Pnt2, 2);
generic_as_bytes_impl!(Vec3, 2);
generic_as_bytes_impl!(Pnt3, 2);

// FIXME: implement for all `T: Copy` insead?
