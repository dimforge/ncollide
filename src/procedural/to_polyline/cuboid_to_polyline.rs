use na::{Pnt2, Vec2};
use na;
use geom::{Cuboid2, Cuboid2d};
use procedural::{ToPolyline, Polyline};
use procedural;

macro_rules! impl_to_polyline_cuboid2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, ()> for $t {
            fn to_polyline(&self, _: ()) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                let _2: $n = na::cast(2.0f64);

                procedural::rectangle(&(self.half_extents() * _2))
            }
        }
    }
)

impl_to_polyline_cuboid2!(Cuboid2, f32)
impl_to_polyline_cuboid2!(Cuboid2d, f64)
