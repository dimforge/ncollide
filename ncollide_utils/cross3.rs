use std::ops::{Index, IndexMut};
use na::{Dim, Zero};
use na;
use math::Scalar;

/// A 3d cross product that do not require the `Cross<Self, Self>` trait impl.
///
/// XXX: this is a HACK until the trait reform is done on rustc. This allows the use of the cross
/// product without the need of the `Cross<Self, Self>` trait impl.
///
/// Fails if the dimension of `V` is not 3.
#[inline]
pub fn cross3<N, V>(a: &V, b: &V) -> V
    where N: Scalar,
          V: Zero + Index<uint, Output = N> + IndexMut<uint, Output = N> + Dim {
    assert!(na::dim::<V>() == 3);

    let mut res = na::zero::<V>();

    res[0] = a[1] * b[2] - a[2] * b[1];
    res[1] = a[2] * b[0] - a[0] * b[2];
    res[2] = a[0] * b[1] - a[1] * b[0];

    res
}
