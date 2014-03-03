use nalgebra::na;
use geom::Mesh;
use volumetric::Volumetric;
use math::{Scalar, Vector, AngularInertia};

impl Volumetric for Mesh {
    #[inline]
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vector, AngularInertia) {
        (na::zero(), na::zero(), na::zero())
    }
}
