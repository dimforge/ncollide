use geom::Convex;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

impl Volumetric for Convex {
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        fail!("Not yet implemented.")
    }
}
