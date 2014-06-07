use nalgebra::na;
use geom::Ball;
use procedural::{ToPolyline, Polyline};
use procedural;
use math::{Scalar, Vect};

impl ToPolyline<u32> for Ball {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline<Scalar, Vect> {
        let diameter: Scalar = self.radius() * na::cast(2.0);

        procedural::circle(&diameter, nsubdiv)
    }
}
