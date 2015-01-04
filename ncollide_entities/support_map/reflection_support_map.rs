use std::ops::Neg;
use support_map::{SupportMap, PreferedSamplingDirections};
use shape::Reflection;
use math::{Scalar, Vect};


impl<'a, N, P, V, M, Sized? G> SupportMap<P, V, M> for Reflection<'a, G>
    where N: Scalar,
          P: Neg<P>,
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
impl<'a, V: Neg<V>, M, Sized? G> PreferedSamplingDirections<V, M> for Reflection<'a, G>
    where G: PreferedSamplingDirections<V, M> {
    /// Applies a function to this shape with a given transform.
    fn sample(&self, m: &M, f: |V| -> ()) {
        self.shape().sample(m, |v| f(-v))
    }
}
