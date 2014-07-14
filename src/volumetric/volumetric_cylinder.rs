use geom::Cylinder;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

#[dim2]
use nalgebra::na::Indexable;
#[dim2]
use nalgebra::na;

#[dim3]
use std::num::Float;
#[dim3]
use nalgebra::na::Indexable;
#[dim3]
use nalgebra::na;

/// Computes the volume of a cylinder.
#[dim2]
#[inline]
pub fn cylinder_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    // same as a rectangle
    half_height * *radius * na::cast(4.0f64)
}

    
/// Computes the volume of a cylinder.
#[dim3]
#[inline]
pub fn cylinder_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    half_height * *radius * *radius * Float::pi() * na::cast(2.0f64)
}

/// Not yet implemented in 4d.
#[dim4]
#[inline]
pub fn cylinder_volume(_: &Scalar, _: &Scalar) -> Scalar {
    fail!("Not yet implemented in 4d.")
}


#[dim2]
impl Volumetric for Cylinder {
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let mass = cylinder_volume(&self.half_height(), &self.radius()) * *density;
        // same as the box
        let _2:   Scalar = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * mass * _2 * _2;
        let ix      = w * self.half_height() * self.half_height();
        let iy      = w * self.radius() * self.radius();

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), ix + iy);

        (mass, na::zero(), res)
    }
}

#[dim3]
impl Volumetric for Cylinder {
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let mass = cylinder_volume(&self.half_height(), &self.radius()) * *density;
        let sq_radius = self.radius() * self.radius();
        let sq_height = self.half_height() * self.half_height() * na::cast(4.0f64);
        let off_principal = mass * (sq_radius * na::cast(3.0f64) + sq_height) / na::cast(12.0f64);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), off_principal.clone());
        res.set((1, 1), mass * sq_radius / na::cast(2.0f64));
        res.set((2, 2), off_principal);

        (mass, na::zero(), res)
    }
}

#[dim4]
impl Volumetric for Cylinder {
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        fail!("mass_properties is not yet implemented for 4d cylinders.")
    }
}
