use geom::GeomWithMargin;
use procedural::{ToPolyline, Polyline};
use math::{Scalar, Vect};


impl<'a, G: ToPolyline<P>, P> ToPolyline<P> for GeomWithMargin<'a, G> {
    fn to_polyline(&self, parameter: P) -> Polyline<Scalar, Vect> {
        self.geom().to_polyline(parameter)
    }
}
