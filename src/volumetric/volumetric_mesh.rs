use nalgebra::na;
use geom::Mesh;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

impl Volumetric for Mesh {
    #[inline]
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        (na::zero(), na::zero(), na::zero())
    }
}
