use nalgebra::na;
use geom::Plane;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

impl Volumetric for Plane {
    #[inline]
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        (na::zero(), na::zero(), na::zero())
    }
}
