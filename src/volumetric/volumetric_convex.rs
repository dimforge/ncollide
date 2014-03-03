use geom::Convex;
use volumetric::Volumetric;
use math::{Scalar, Vector, AngularInertia};

impl Volumetric for Convex {
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vector, AngularInertia) {
        fail!("Not yet implemented.")
    }
}
