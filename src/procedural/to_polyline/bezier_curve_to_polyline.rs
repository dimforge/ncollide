use na::{Pnt2, Vec2};
use shape::BezierCurve2;
use procedural::{ToPolyline, Polyline};
use procedural;
use math::Scalar;

impl<N: Scalar> ToPolyline<N, Pnt2<N>, Vec2<N>, u32> for BezierCurve2<N> {
    fn to_polyline(&self, nsubdiv: u32) -> Polyline<N, Pnt2<N>, Vec2<N>> {
        procedural::bezier_curve(self.control_points(), nsubdiv as uint)
    }
}
