use nalgebra::na;
use geom::Triangle;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

impl Volumetric for Triangle {
    #[inline]
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        assert!(na::dim::<Vect>() > 2, "Volumetric implementation for 2d triangles is not implemented yet.");
        (na::zero(), na::zero(), na::zero())
    }
}
