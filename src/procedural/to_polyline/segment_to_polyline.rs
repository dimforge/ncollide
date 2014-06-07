use std::num::Zero;
use procedural::{ToPolyline, Polyline};
use geom::Segment;
use math::{Scalar, Vect};

impl ToPolyline<()> for Segment {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Non-zero margin is not yet suported.");
        Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
    }
}
