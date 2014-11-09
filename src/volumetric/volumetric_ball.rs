use std::num::Zero;
use std::num;
use na::Orig;
use na::{Pnt2, Pnt3, Mat1, Mat3};
use na;
use volumetric::Volumetric;
use shape::{Ball2, Ball3};
use math::Scalar;


/// The volume of a ball.
#[inline]
pub fn ball_volume<N: Scalar>(dim: uint, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    let _pi: N = Float::pi();
    _pi * num::pow(radius.clone(), dim)
}

/// The surface of a ball.
#[inline]
pub fn ball_surface<N: Scalar>(dim: uint, radius: N) -> N {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            let _pi: N = Float::pi();
            _pi * radius * na::cast(2.0f64)
        }
        3 => {
            let _pi: N = Float::pi();
            _pi * radius * radius * na::cast(4.0f64)
        }
        _ => unreachable!()
    }
}

/// The center of mass of a ball.
#[inline]
pub fn ball_center_of_mass<P: Orig>() -> P {
    na::orig()
}

/// The unit angular inertia of a ball.
#[inline]
pub fn ball_unit_angular_inertia<N, I>(dim: uint, radius: N) -> I
    where N: Scalar,
          I: Zero + IndexMut<(uint, uint), N> {
    assert!(dim == 2 || dim == 3);

    match dim {
        2 => {
            let diag = radius * radius / na::cast(2.0f64);
            let mut res = na::zero::<I>();

            res[(0, 0)] = diag;

            res
        }
        3 => {
            let diag: N = radius * radius * na::cast(2.0f64 / 5.0);
            let mut res = na::zero::<I>();

            res[(0, 0)] = diag.clone();
            res[(1, 1)] = diag.clone();
            res[(2, 2)] = diag.clone();

            res
        }
        _ => unreachable!()
    }
}

macro_rules! impl_volumetric_ball(
    ($t: ident, $dim: expr, $p: ident, $i: ident) => {
        impl<N: Scalar> Volumetric<N, $p<N>, $i<N>> for $t<N> {
            fn surface(&self) -> N {
                ball_surface($dim, self.radius())
            }

            fn volume(&self) -> N {
                ball_volume($dim, self.radius())
            }

            fn center_of_mass(&self) -> $p<N> {
                ball_center_of_mass()
            }

            fn unit_angular_inertia(&self) -> $i<N> {
                ball_unit_angular_inertia($dim, self.radius())
            }
        }
    }
)

impl_volumetric_ball!(Ball2, 2, Pnt2, Mat1)
impl_volumetric_ball!(Ball3, 3, Pnt3, Mat3)
