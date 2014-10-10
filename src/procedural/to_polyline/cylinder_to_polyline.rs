use na::Vec2;
use na;
use geom::Cylinder;
use procedural::{Polyline, ToPolyline};
use procedural;
use math::{Scalar, Point, Vect};


#[cfg(feature = "2d")]
impl ToPolyline<()> for Cylinder {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Point, Vect> {
        let _2: Scalar = na::cast(2.0f64);

        procedural::rectangle(&Vec2::new(self.radius() * _2, self.half_height() * _2))
    }
}
