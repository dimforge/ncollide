use geom::Cone;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};
use nalgebra::na;
use nalgebra::na::Indexable;

// #[dim3]
use std::num::Float;

/// Computes the volume of a cone.
#[dim2]
#[inline]
pub fn cone_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    // same as a isosceles triangle
    *radius * *half_height * na::cast(2.0f64)
}

/// Computes the volume of a cone.
#[dim3]
#[inline]
pub fn cone_volume(half_height: &Scalar, radius: &Scalar) -> Scalar {
    *radius * *radius * Float::pi() * *half_height * na::cast(2.0f64 / 3.0)
}

/// Not yet implemented in 4d.
#[dim4]
#[inline]
pub fn cone_volume(_: &Scalar, _: &Scalar) -> Scalar {
    fail!("Not yet impelmented in 4d.")
}

#[dim2]
impl Volumetric for Cone {
    fn surface(&self) -> Scalar {
        let height = self.half_height() * na::cast(2.0f64);
        let side   = (height * height + self.radius() * self.radius()).sqrt();

        self.radius() * na::cast(2.0f64) + side
    }

    fn volume(&self) -> Scalar {
        cone_volume(&self.half_height(), &self.radius())
    }

    fn center_of_mass(&self) -> Vect {
        let mut com: Vect = na::zero();
        com.set(1, -self.half_height() / na::cast(2.0f64));

        com
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        // FIXME: not sure about that…
        let mut res: AngularInertia = na::zero();

        res.set((0, 0),
                self.radius() * self.half_height() * self.half_height() * self.half_height()
                / na::cast(3.0f64));

        res
    }
}

#[dim3]
impl Volumetric for Cone {
    fn surface(&self) -> Scalar {
        let _pi    = Float::pi();
        let height = self.half_height() + self.half_height();
        let side   = (height * height + self.radius() * self.radius()).sqrt();

        self.radius() * self.radius() *_pi + side * self.radius() * _pi
    }

    fn volume(&self) -> Scalar {
        cone_volume(&self.half_height(), &self.radius())
    }

    fn center_of_mass(&self) -> Vect {
        let mut com: Vect = na::zero();
        com.set(1, -self.half_height() / na::cast(2.0f64));

        com
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let sq_radius = self.radius() * self.radius();
        let sq_height = self.half_height() * self.half_height() *
                          na::cast(4.0f64);
        let off_principal = sq_radius * na::cast(3.0f64 / 20.0) +
                            sq_height * na::cast(3.0f64 / 5.0);

        let principal = sq_radius * na::cast(3.0f64 / 10.0);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), off_principal.clone());
        res.set((1, 1), principal);
        res.set((2, 2), off_principal);

        res
    }
}
