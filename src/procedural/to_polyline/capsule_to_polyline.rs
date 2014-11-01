use na::{Pnt2, Vec2};
use na;
use shape::{Capsule2, Capsule2d};
use procedural::{ToPolyline, Polyline};
use procedural::utils;

macro_rules! impl_to_polyline_capsule2(
    ($t: ty, $n: ty) => {
        impl ToPolyline<$n, Pnt2<$n>, Vec2<$n>, u32> for $t {
            fn to_polyline(&self, nsubdiv: u32) -> Polyline<$n, Pnt2<$n>, Vec2<$n>> {
                let pi: $n = Float::pi();
                let dtheta     = pi / na::cast(nsubdiv as f64);

                let mut points: Vec<Pnt2<$n>> = Vec::with_capacity(nsubdiv as uint);

                utils::push_xy_arc(self.radius(), nsubdiv, dtheta, &mut points);

                let npoints = points.len();

                for i in range(0, npoints) {
                    let new_point = points[i] + Vec2::new(na::zero(), self.half_height());

                    points.push(-new_point);
                    *points.get_mut(i) = new_point;
                }

                Polyline::new(points, None)
            }
        }
    }
)

impl_to_polyline_capsule2!(Capsule2, f32)
impl_to_polyline_capsule2!(Capsule2d, f64)
