use std::num::Float;
use std::num;
use na;
use geom::Ball;
use volumetric::Volumetric;
use math::{Scalar, Point, AngularInertia};

#[cfg(not(feature = "4d"))]
use na::Indexable;

/// Computes the volume of a ball.
#[inline]
pub fn ball_volume(radius: &Scalar) -> Scalar {
    let _pi: Scalar = Float::pi();
    _pi * num::pow(radius.clone(), na::dim::<Point>())
}

#[cfg(feature = "2d")]
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
    fn center_of_mass(&self) -> Point {
        na::orig()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let diag = self.radius() * self.radius() / na::cast(2.0f64);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), diag);

        res
    }
}

#[cfg(feature = "3d")]
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
    fn center_of_mass(&self) -> Point {
        na::orig()
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
