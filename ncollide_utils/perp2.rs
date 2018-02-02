use alga::linear::FiniteDimInnerSpace;
use na;

/// A 2d perpendicular product that does fot have compile-time restrictions on the vector dimension..
#[inline]
pub fn perp2<V: FiniteDimInnerSpace>(a: &V, b: &V) -> V::Real {
    assert!(na::dimension::<V>() == 2);

    a[0] * b[1] - a[1] * b[0]
}
