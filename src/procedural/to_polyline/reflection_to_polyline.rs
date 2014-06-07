use geom::Reflection;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Vect};


impl<'a, G: ToPolyline<P>, P> ToPolyline<P> for Reflection<'a, G> {
    fn to_polyline(&self, parameter: P) -> Polyline<Scalar, Vect> {
        let mut res = self.geom().to_polyline(parameter);

        for c in res.coords.mut_iter() {
            *c = -*c;
        }

        res
    }
}
