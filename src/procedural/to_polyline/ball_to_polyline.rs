use na;
use geom::Ball;
use procedural::{ToPolyline, Polyline};
use procedural;
use math::{Scalar, Point, Vect};

impl ToPolyline<u32> for Ball {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline<Scalar, Point, Vect> {
        let diameter: Scalar = self.radius() * na::cast(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
