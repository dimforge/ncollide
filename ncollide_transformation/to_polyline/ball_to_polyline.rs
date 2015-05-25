use na::Pnt2;
use na;
use math::Scalar;
use entities::shape::Ball2;
use procedural::Polyline2;
use procedural;
use super::ToPolyline;

impl<N: Scalar> ToPolyline<Pnt2<N>, u32> for Ball2<N> {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline2<N> {
        let diameter = self.radius() * na::cast(2.0f64);

        procedural::circle(&diameter, nsubdiv)
    }
}
