use na::{Pnt2, Vec2};
use na;
use shape::{Ball2, Ball2d};
use procedural::{ToPolyline, Polyline};
use procedural;

macro_rules! impl_to_polyline_ball2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, u32> for $t {
            fn to_polyline(&self, nsubdiv: u32) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                let diameter = self.radius() * na::cast(2.0f64);

                procedural::circle(&diameter, nsubdiv)
            }
        }
    }
)

impl_to_polyline_ball2!(Ball2, f32)
impl_to_polyline_ball2!(Ball2d, f64)
