use na::Indexable;
use na;
use geom::Cylinder;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

#[cfg(feature = "3d")]
use std::num::Float;

/// Computes the volume of a cylinder.
#[cfg(feature = "2d")]
#[inline]
pub fn cylinder_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    // same as a rectangle
    half_height * *radius * na::cast(4.0f64)
}

    
/// Computes the volume of a cylinder.
#[cfg(feature = "3d")]
#[inline]
pub fn cylinder_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    half_height * *radius * *radius * Float::pi() * na::cast(2.0f64)
}

/// Not yet implemented in 4d.
#[cfg(feature = "4d")]
#[inline]
pub fn cylinder_volume(_: &Scalar, _: &Scalar) -> Scalar {
    fail!("Not yet implemented in 4d.")
}


#[cfg(feature = "2d")]
impl Volumetric for Cylinder {
    #[inline]
    fn surface(&self) -> Scalar {
        (self.half_height() + self.radius()) * na::cast(2.0f64)
    }

    #[inline]
    fn volume(&self) -> Scalar {
        cylinder_volume(&self.half_height(), &self.radius())
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        // Same a the rectangle.
        let _2:   Scalar = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * _2 * _2;
        let ix      = w * self.half_height() * self.half_height();
        let iy      = w * self.radius() * self.radius();

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), ix + iy);

        res
    }
}

#[cfg(feature = "3d")]
impl Volumetric for Cylinder {
    #[inline]
    fn surface(&self) -> Scalar {
        let _pi: Scalar = Float::pi();
        let basis = self.radius() * self.radius() * _pi;
        let side  = _pi * self.radius() * (self.half_height() + self.half_height()) * na::cast(2.0f64);

        side + basis + basis
    }

    #[inline]
    fn volume(&self) -> Scalar {
        cylinder_volume(&self.half_height(), &self.radius())
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let sq_radius = self.radius() * self.radius();
        let sq_height = self.half_height() * self.half_height() * na::cast(4.0f64);
        let off_principal = (sq_radius * na::cast(3.0f64) + sq_height) / na::cast(12.0f64);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), off_principal.clone());
        res.set((1, 1), sq_radius / na::cast(2.0f64));
        res.set((2, 2), off_principal);

        res
    }
}
