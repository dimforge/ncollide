use na;
use alga::linear::FiniteDimVectorSpace;

// FIXME: move this to alga?
/// Creates a vector with all its components equal to `e`.
#[inline]
pub fn repeat<V: FiniteDimVectorSpace>(e: V::Field) -> V {
    let mut res: V = na::zero();

    for i in 0..V::dimension() {
        res[i] = e.clone()
    }

    res
}
