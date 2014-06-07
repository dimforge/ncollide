use std::num::Zero;
use procedural::{ToPolyline, Polyline};
use geom::Triangle;
use math::{Scalar, Vect};

impl ToPolyline<()> for Triangle {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Non-zero margins are not supported yet.");

        Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
    }
}
