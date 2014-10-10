use geom::Reflection;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Point, Vect};


impl<'a, G: ToPolyline<P>, P> ToPolyline<P> for Reflection<'a, G> {
    fn to_polyline(&self, parameter: P) -> Polyline<Scalar, Point, Vect> {
        let mut res = self.geom().to_polyline(parameter);

        for c in res.coords.iter_mut() {
            *c = -*c;
        }

        res
    }
}
