use procedural::{ToPolyline, Polyline};
use geom::Segment;
use math::{Scalar, Point, Vect};

impl ToPolyline<()> for Segment {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Point, Vect> {
        Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
    }
}
