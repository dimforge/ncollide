use geom::Cylinder;
use volumetric::Volumetric;
use math::{N, V, II};

#[cfg(dim2)]
use nalgebra::na::Indexable;
#[cfg(dim2)]
use nalgebra::na;

#[cfg(dim3)]
use std::num::Float;
#[cfg(dim3)]
use nalgebra::na::Indexable;
#[cfg(dim3)]
use nalgebra::na;

/// Computes the volume of a cylinder.
#[cfg(dim2)]
#[inline]
pub fn cylinder_volume(half_height: &N, radius: &N) -> N {
    // same as a rectangle
    half_height * *radius * na::cast(4.0)
}

    
/// Computes the volume of a cylinder.
#[cfg(dim3)]
#[inline]
pub fn cylinder_volume(half_height: &N, radius: &N) -> N {
    half_height * *radius * *radius * Float::pi() * na::cast(2.0)
}

#[cfg(dim2)]
impl Volumetric for Cylinder {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass = cylinder_volume(&self.half_height(), &self.radius()) * *density;
        // same as the box
        let _2:   N = na::cast(2.0);
        let _i12: N = na::cast(1.0 / 12.0);
        let w       = _i12 * mass * _2 * _2;
        let ix      = w * self.half_height() * self.half_height();
        let iy      = w * self.radius() * self.radius();

        let mut res: II = na::zero();

        res.set((0, 0), ix + iy);

        (mass, na::zero(), res)
    }
}

#[cfg(dim3)]
impl Volumetric for Cylinder {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass = cylinder_volume(&self.half_height(), &self.radius()) * *density;
        let sq_radius = self.radius() * self.radius();
        let sq_height = self.half_height() * self.half_height() * na::cast(4.0);
        let off_principal = mass * (sq_radius * na::cast(3.0) + sq_height) / na::cast(12.0);

        let mut res: II = na::zero();

        res.set((0, 0), mass * sq_radius / na::cast(2.0));
        res.set((1, 1), off_principal.clone());
        res.set((2, 2), off_principal);

        (mass, na::zero(), res)
    }
}

#[cfg(dim4)]
impl Volumetric for Cylinder {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("mass_properties is not yet implemented for 4d cylinders.")
    }
}
