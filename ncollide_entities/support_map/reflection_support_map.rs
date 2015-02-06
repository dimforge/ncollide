use std::ops::Neg;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Reflection;
use math::{Scalar, Vect};


#[old_impl_check]
impl<'a, N, P, V, M, G: ?Sized> SupportMap<P, V, M> for Reflection<'a, G>
    where N: Scalar,
          P: Neg<Output = P>,
          V: Vect<N>,
          G: SupportMap<P, V, M> {
    #[inline]
    fn support_point(&self, m: &M, dir: &V) -> P {
        -self.shape().support_point(m, &-*dir)
    }
}

/// Trait of shapes having prefered sampling directions for the Minkowski sampling algorithm.
///
/// Those directions are usually the shape faces normals.
impl<'a, V: Neg<Output = V>, M, G: ?Sized> PreferedSamplingDirections<V, M> for Reflection<'a, G>
    where G: PreferedSamplingDirections<V, M> {
    /// Applies a function to this shape with a given transform.
    fn sample(&self, m: &M, f: &mut FnMut(V)) {
        self.shape().sample(m, &mut |v: V| f(-v));
    }
}
