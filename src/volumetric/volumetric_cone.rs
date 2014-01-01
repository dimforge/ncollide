use std::num::Real;
use nalgebra::na::{Cast, Indexable};
use nalgebra::na;
use geom::Cone;
use volumetric::Volumetric;
use math::{N, V, II};

#[inline]
pub fn cone_volume(half_height: &N,
                   radius:      &N,
                   dim:         uint)
                   -> N {
    if dim == 2 {
        // same as a isosceles triangle
        *radius * *half_height * Cast::from(2.0)
    }
    else if dim == 3 {
        *radius * *radius * Real::pi() * *half_height * Cast::from(2.0 / 3.0)
    }
    else {
        fail!("Volume for n-dimensional cone, n > 3, is not implemented.")
    }
}

impl Volumetric for Cone {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = na::dim::<V>();
        let mass = cone_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // FIXME: not sure about that…
            let mut res: II = na::zero();

            res.set(
                (0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                              / Cast::from(3.0)
            );

            let mut center: V = na::zero();
            center.set(0, -self.half_height() / Cast::from(2.0));

            (mass, center, res)
        }
        else if dim == 3 {
            let m_sq_radius = mass * self.radius() * self.radius();
            let m_sq_height = mass * self.half_height() * self.half_height() *
                                     Cast::from(4.0);
            let off_principal = m_sq_radius * Cast::from(3.0 / 20.0) +
                                m_sq_height * Cast::from(3.0 / 5.0);

            let principal = m_sq_radius * Cast::from(3.0 / 10.0);

            let mut res: II = na::zero();

            res.set((0, 0), principal);
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            let mut center: V = na::zero();
            center.set(0, -self.half_height() / Cast::from(2.0));

            (mass, center, res)
        }

        else {
            fail!("Inertia tensor for n-dimensional cone, n > 3, is not implemented.")
        }
    }
}
