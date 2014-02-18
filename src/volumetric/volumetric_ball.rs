use std::num::Float;
use std::num;
use nalgebra::na;
use geom::Ball;
use volumetric::Volumetric;
use math::{N, V, II};

#[cfg(dim2)]
use nalgebra::na::Indexable;

#[cfg(dim3)]
use nalgebra::na::Indexable;

/// Computes the volume of a ball.
#[inline]
pub fn ball_volume(radius: &N) -> N {
    let _pi: N = Float::pi();
    _pi * num::pow(radius.clone(), na::dim::<V>())
}

#[cfg(dim2)]
impl Volumetric for Ball {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume = ball_volume(&self.radius());
        let mass   = volume * *density;
        let diag   = self.radius() * self.radius() * mass / na::cast(2.0);

        let mut res: II = na::zero();

        res.set((0, 0), diag);

        (mass, na::zero(), res)
    }
}

#[cfg(dim3)]
impl Volumetric for Ball {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume  = ball_volume(&self.radius());
        let mass    = volume * *density;
        let diag: N = mass                *
                      na::cast(2.0 / 5.0) *
                      self.radius()       *
                      self.radius();

        let mut res: II = na::zero();

        res.set((0, 0), diag.clone());
        res.set((1, 1), diag.clone());
        res.set((2, 2), diag.clone());

        (mass, na::zero(), res)
    }
}

#[cfg(dim4)]
impl Volumetric for Ball {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("mass_properties is not yet implemented for 4d balls")
    }
}

