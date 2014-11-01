use na::{Pnt2, Vec2};
use shape::{Segment2, Segment2d};
use procedural::{ToPolyline, Polyline};

macro_rules! impl_to_polyline_segment2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, ()> for $t {
            fn to_polyline(&self, _: ()) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                Polyline::new(vec!(self.a().clone(), self.b().clone()), None)
            }
        }
    }
)

impl_to_polyline_segment2!(Segment2, f32)
impl_to_polyline_segment2!(Segment2d, f64)
