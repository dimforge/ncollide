use geom::Cone;
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

/// Computes the volume of a cone.
#[cfg(dim2)]
#[inline]
pub fn cone_volume(half_height: &N, radius: &N) -> N {
    // same as a isosceles triangle
    *radius * *half_height * na::cast(2.0)
}

/// Computes the volume of a cone.
#[cfg(dim3)]
#[inline]
pub fn cone_volume(half_height: &N, radius: &N) -> N {
    *radius * *radius * Float::pi() * *half_height * na::cast(2.0 / 3.0)
}

#[cfg(dim2)]
impl Volumetric for Cone {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass = cone_volume(&self.half_height(), &self.radius()) * *density;

        // FIXME: not sure about that…
        let mut res: II = na::zero();

        res.set(
            (0, 0),
            self.radius() * self.half_height() * self.half_height() * self.half_height()
            / na::cast(3.0)
            );

        let mut center: V = na::zero();
        center.set(0, -self.half_height() / na::cast(2.0));

        (mass, center, res)
    }
}

#[cfg(dim3)]
impl Volumetric for Cone {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let mass        = cone_volume(&self.half_height(), &self.radius()) * *density;
        let m_sq_radius = mass * self.radius() * self.radius();
        let m_sq_height = mass * self.half_height() * self.half_height() *
                          na::cast(4.0);
        let off_principal = m_sq_radius * na::cast(3.0 / 20.0) +
                            m_sq_height * na::cast(3.0 / 5.0);

        let principal = m_sq_radius * na::cast(3.0 / 10.0);

        let mut res: II = na::zero();

        res.set((0, 0), principal);
        res.set((1, 1), off_principal.clone());
        res.set((2, 2), off_principal);

        let mut center: V = na::zero();
        center.set(0, -self.half_height() / na::cast(2.0));

        (mass, center, res)
    }
}

#[cfg(dim4)]
impl Volumetric for Cone {
    fn mass_properties(&self, _: &N) -> (N, V, II) {
        fail!("mass_properties is not yet implemented for cones.")
    }
}
