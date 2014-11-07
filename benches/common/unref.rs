use ncollide::math::Scalar;

pub trait Unref<T> {
    fn unref(a: Self) -> T;
}

impl<'a, N: Scalar> Unref<N> for &'a N {
    #[inline(always)]
    fn unref(a: &N) -> N {
        *a
    }
}

impl<'a> Unref<bool> for &'a bool {
    #[inline(always)]
    fn unref(a: &bool) -> bool {
        *a
    }
}

impl<'a, T> Unref<&'a T> for &'a T {
    #[inline(always)]
    fn unref(a: &'a T) -> &'a T {
        a
    }
}

#[inline(always)]
pub fn unref<T: Unref<O>, O>(val: T) -> O {
    Unref::unref(val)
}
