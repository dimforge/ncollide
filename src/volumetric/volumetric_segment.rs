use nalgebra::na;
use geom::Segment;
use volumetric::Volumetric;
use math::{Scalar, Vector, AngularInertia};

impl Volumetric for Segment {
    #[inline]
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vector, AngularInertia) {
        (na::zero(), na::zero(), na::zero())
    }
}
