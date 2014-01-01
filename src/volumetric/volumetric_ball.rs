use std::num::Real;
use nalgebra::na::{Cast, Indexable};
use nalgebra::na;
use geom::Ball;
use volumetric::Volumetric;
use math::{N, V, II};

#[inline]
pub fn ball_volume(radius: &N, dim: uint) -> N {
    let _pi: N = Real::pi();
    _pi * radius.pow(&Cast::from(dim as f32))
}

impl Volumetric for Ball {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let volume = ball_volume(&self.radius(), na::dim::<V>());
        let mass   = volume * *density;

        let dim = na::dim::<V>();

        if dim == 2 {
            let diag = self.radius() * self.radius() * mass / Cast::from(2.0);

            let mut res: II = na::zero();

            res.set((0, 0), diag);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let _0: N  = na::zero();
            let diag: N = mass                  *
                          Cast::from(2.0 / 5.0) *
                          self.radius()         *
                          self.radius();

            let mut res: II = na::zero();

            res.set((0, 0), diag.clone());
            res.set((1, 1), diag.clone());
            res.set((2, 2), diag.clone());

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional balls, n > 3, is not implemented.")
        }
    }
}

