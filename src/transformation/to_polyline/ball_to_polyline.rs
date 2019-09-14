use super::ToPolyline;
use crate::procedural::{self, Polyline};
use crate::shape::Ball;
use alga::general::RealField;
use na;

impl<N: RealField> ToPolyline<N> for Ball<N> {
    type DiscretizationParameter = u32;

    fn to_polyline(&self, nsubdiv: u32) -> Polyline<N> {
        let diameter = self.radius() * na::convert(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
