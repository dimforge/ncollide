use std::num::Real;
use nalgebra::na::{Cast, Indexable};
use nalgebra::na;
use geom::Cylinder;
use volumetric::Volumetric;
use math::{N, V, II};

#[inline]
pub fn cylinder_volume(half_height: &N, radius: &N, dim: uint) -> N {
    if dim == 2 {
        // same as a rectangle
        half_height * *radius * Cast::from(4.0)
    }
    else if dim == 3 {
        half_height * *radius * *radius * Real::pi() * Cast::from(2.0)
    }
    else {
        fail!("Volume for n-dimensional cylinders, n > 3, is not implemented.")
    }
}

impl Volumetric for Cylinder {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let dim  = na::dim::<V>();
        let mass = cylinder_volume(&self.half_height(), &self.radius(), dim) * *density;

        if dim == 2 {
            // same as the box
            let _2:   N = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * self.half_height() * self.half_height();
            let iy      = w * self.radius() * self.radius();

            let mut res: II = na::zero();

            res.set((0, 0), ix + iy);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let sq_radius = self.radius() * self.radius();
            let sq_height = self.half_height() * self.half_height() * Cast::from(4.0);
            let off_principal = mass * (sq_radius * Cast::from(3.0) + sq_height) / Cast::from(12.0);

            let mut res: II = na::zero();

            res.set((0, 0), mass * sq_radius / Cast::from(2.0));
            res.set((1, 1), off_principal.clone());
            res.set((2, 2), off_principal);

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional cylinder, n > 3, is not implemented.")
        }
    }
}

