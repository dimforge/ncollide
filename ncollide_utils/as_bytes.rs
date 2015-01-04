use std::mem;
use na::{Vec2, Vec3, Pnt2, Pnt3};

/// Trait that transforms thing to a slice of u8.
pub trait AsBytes {
    fn as_bytes<'a>(&'a self) -> &'a [u8];
}

impl AsBytes for Vec3<f32> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Vec3<f32>, &'a [u8; 12]>(self).as_slice()
        }
    }
}

impl AsBytes for Vec3<f64> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Vec3<f64>, &'a [u8; 24]>(self).as_slice()
        }
    }
}

impl AsBytes for Vec2<f32> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Vec2<f32>, &'a [u8; 8]>(self).as_slice()
        }
    }
}

impl AsBytes for Vec2<f64> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Vec2<f64>, &'a [u8; 16]>(self).as_slice()
        }
    }
}

impl AsBytes for Pnt3<f32> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Pnt3<f32>, &'a [u8; 12]>(self).as_slice()
        }
    }
}

impl AsBytes for Pnt3<f64> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Pnt3<f64>, &'a [u8; 24]>(self).as_slice()
        }
    }
}

impl AsBytes for Pnt2<f32> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Pnt2<f32>, &'a [u8; 8]>(self).as_slice()
        }
    }
}

impl AsBytes for Pnt2<f64> {
    #[inline(always)]
    fn as_bytes<'a>(&'a self) -> &'a [u8] {
        unsafe {
            mem::transmute::<&'a Pnt2<f64>, &'a [u8; 16]>(self).as_slice()
        }
    }
}

// FIXME: implement for other things.
