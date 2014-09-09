use std::num::Zero;
use procedural::{ToPolyline, Polyline};
use geom::Segment;
use math::{Scalar, Vect};

impl ToPolyline<()> for Segment {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
    }
}
