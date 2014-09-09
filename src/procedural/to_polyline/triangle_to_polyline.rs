use std::num::Zero;
use procedural::{ToPolyline, Polyline};
use geom::Triangle;
use math::{Scalar, Vect};

impl ToPolyline<()> for Triangle {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
    }
}
