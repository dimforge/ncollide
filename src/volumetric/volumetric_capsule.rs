use geom::Capsule;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

#[cfg(dim2)]
use volumetric::cylinder_volume;
#[cfg(dim2)]
use volumetric::ball_volume;

#[cfg(dim3)]
use volumetric::cylinder_volume;
#[cfg(dim3)]
use volumetric::ball_volume;

/// Computes the volume of a capsule.
#[inline]
#[cfg(dim2)]
pub fn capsule_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    cylinder_volume(half_height, radius) + ball_volume(radius)
}

/// Computes the volume of a capsule.
#[inline]
#[cfg(dim3)]
pub fn capsule_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    cylinder_volume(half_height, radius) + ball_volume(radius)
}

impl Volumetric for Capsule {
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        fail!("Not yet implemented.")
    }
}
