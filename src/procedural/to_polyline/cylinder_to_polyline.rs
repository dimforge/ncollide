use na::{Pnt2, Vec2};
use na;
use shape::{Cylinder2, Cylinder2d};
use procedural::{ToPolyline, Polyline};
use procedural;

macro_rules! impl_to_polyline_cylinder2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, ()> for $t {
            fn to_polyline(&self, _: ()) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                let _2: $n = na::cast(2.0f64);

                procedural::rectangle(&Vec2::new(self.radius() * _2, self.half_height() * _2))
            }
        }
    }
)

impl_to_polyline_cylinder2!(Cylinder2, f32)
impl_to_polyline_cylinder2!(Cylinder2d, f64)
