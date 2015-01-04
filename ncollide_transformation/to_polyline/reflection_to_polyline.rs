use entities::shape::Reflection;
use math::{Scalar, Point, Vect};
use procedural::Polyline;
use super::ToPolyline;

impl<'a, N, P, V, G: ToPolyline<N, P, V, I>, I> ToPolyline<N, P, V, I> for Reflection<'a, G>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn to_polyline(&self, parameter: I) -> Polyline<N, P, V> {
        let mut res = self.shape().to_polyline(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
