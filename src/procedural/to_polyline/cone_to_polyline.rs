use na::{Pnt2, Vec2};
use na;
use shape::{Cone2, Cone2d};
use procedural::{ToPolyline, Polyline};

macro_rules! impl_to_polyline_cone2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, ()> for $t {
            fn to_polyline(&self, _: ()) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                let hh = self.half_height();
                let r  = self.radius();

                let coords = vec!(Pnt2::new(-r, -hh), Pnt2::new(r, -hh), Pnt2::new(na::zero(), hh));

                Polyline::new(coords, None)
            }
        }
    }
)

impl_to_polyline_cone2!(Cone2, f32)
impl_to_polyline_cone2!(Cone2d, f64)
