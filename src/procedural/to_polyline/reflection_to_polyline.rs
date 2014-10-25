use geom::Reflection;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Point, Vect};

impl<'a, N, P, V, G: ToPolyline<N, P, V, I>, I> ToPolyline<N, P, V, I> for Reflection<'a, G>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    fn to_polyline(&self, parameter: I) -> Polyline<N, P, V> {
        let mut res = self.geom().to_polyline(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
