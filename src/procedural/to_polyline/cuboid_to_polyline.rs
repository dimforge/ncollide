use std::num::Zero;
use nalgebra::na;
use geom::Cuboid;
use procedural::{Polyline, ToPolyline};
use procedural;
use math::{Scalar, Vect};


impl ToPolyline<()> for Cuboid {
    fn to_polyline(&self, _: ()) -> Polyline<Scalar, Vect> {
        assert!(self.margin().is_zero(), "Rounded cuboid polyline generation is not implemented yet.");
        let _2: Scalar = na::cast(2.0);

        procedural::rectangle(&(self.half_extents() * _2))
    }
}
