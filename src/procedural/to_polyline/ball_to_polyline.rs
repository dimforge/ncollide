use na::{Pnt2, Vec2};
use na;
use shape::Ball2;
use procedural::{ToPolyline, Polyline};
use procedural;
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, u32> for Ball2<N> {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline<N, Pnt2<N>, Vec2<N>> {
        let diameter = self.radius() * na::cast(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
