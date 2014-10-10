use procedural::{ToPolyline, Polyline};
use geom::Triangle;
use math::{Scalar, Point, Vect};

impl ToPolyline<()> for Triangle {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Point, Vect> {
        Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
    }
}
