use na::{Pnt2, Vec2};
use shape::{BezierCurve2, BezierCurve2d};
use procedural::{ToPolyline, Polyline};
use procedural;

macro_rules! impl_to_polyline_bezier_curve2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, u32> for $t {
            fn to_polyline(&self, nsubdiv: u32) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                procedural::bezier_curve(self.control_points(), nsubdiv as uint)
            }
        }
    }
)

impl_to_polyline_bezier_curve2!(BezierCurve2, f32)
impl_to_polyline_bezier_curve2!(BezierCurve2d, f64)
