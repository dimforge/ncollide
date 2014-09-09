use std::num::Zero;
use nalgebra::na::Vec2;
use nalgebra::na;
use geom::Cylinder;
use procedural::{Polyline, ToPolyline};
use procedural;
use math::{Scalar, Vect};


#[dim2]
impl ToPolyline<()> for Cylinder {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        let _2: Scalar = na::cast(2.0f64);

        procedural::rectangle(&Vec2::new(self.radius() * _2, self.half_height() * _2))
    }
}
