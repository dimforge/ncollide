use super::ToPolyline;
use alga::general::Real;
use na;
use procedural::{self, Polyline};
use shape::Ball;

impl<N: Real> ToPolyline<N> for Ball<N> {
    type DiscretizationParameter = u32;

    fn to_polyline(&self, nsubdiv: u32) -> Polyline<N> {
        let diameter = self.radius() * na::convert(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
