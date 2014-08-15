use std::num::Float;
use std::num;
use nalgebra::na;
use nalgebra::na::Indexable;
use geom::Ball;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

/// Computes the volume of a ball.
#[inline]
pub fn ball_volume(radius: &Scalar) -> Scalar {
    let _pi: Scalar = Float::pi();
    _pi * num::pow(radius.clone(), na::dim::<Vect>())
}

#[dim2]
impl Volumetric for Ball {
    #[inline]
    fn surface(&self) -> Scalar {
        let _pi: Scalar = Float::pi();
        _pi * self.radius() * na::cast(2.0f64)
    }

    #[inline]
    fn volume(&self) -> Scalar {
        ball_volume(&self.radius())
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let diag = self.radius() * self.radius() / na::cast(2.0f64);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), diag);

        res
    }
}

#[dim3]
impl Volumetric for Ball {
    #[inline]
    fn surface(&self) -> Scalar {
        let _pi: Scalar = Float::pi();
        _pi * self.radius() * self.radius() * na::cast(4.0f64)
    }

    #[inline]
    fn volume(&self) -> Scalar {
        ball_volume(&self.radius())
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let diag: Scalar = self.radius() * self.radius() * na::cast(2.0f64 / 5.0);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), diag.clone());
        res.set((1, 1), diag.clone());
        res.set((2, 2), diag.clone());

        res
    }
}
