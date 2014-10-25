use na::{Pnt2, Vec2};
use geom::{Triangle2, Triangle2d};
use procedural::{ToPolyline, Polyline};

macro_rules! impl_to_polyline_triangle2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, ()> for $t {
            fn to_polyline(&self, _: ()) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                Polyline::new(vec!(self.a().clone(), self.b().clone(), self.c().clone()), None)
            }
        }
    }
)

impl_to_polyline_triangle2!(Triangle2, f32)
impl_to_polyline_triangle2!(Triangle2d, f64)
