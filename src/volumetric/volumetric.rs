//! Traits to compute inertial properties.

use math::{Scalar, Vect, Orientation, Matrix, AngularInertia};

#[dim2]
use nalgebra::na::Mat1;
#[dim2]
use nalgebra::na;

#[dim3]
use nalgebra::na::Mat3;
#[dim3]
use nalgebra::na;

/// Trait to be implemented by inertia tensors.
pub trait InertiaTensor {
    /// Applies this inertia tensor to a vector.
    ///
    /// This is usually done by a matrix-vector multiplication.
    fn apply(&self, a: &Orientation) -> Orientation;

    /// Transforms this inertia tensor from local space to world space.
    fn to_world_space(&self, &Matrix) -> Self;

    /// Computes this inertia tensor relative to a given point.
    fn to_relative_wrt_point(&self, &Scalar, &Vect) -> Self;
}

/// Trait to be implemented by objects which have a mass, a center of mass, and an inverse
/// inertia tensor.
pub trait Volumetric {
    /// Given a density, this computes the mass, center of mass, and inertia tensor of this object.
    fn mass_properties(&self, &Scalar) -> (Scalar, Vect, AngularInertia);
}

#[dim2]
impl InertiaTensor for AngularInertia {
    #[inline]
    fn apply(&self, av: &Orientation) -> Orientation {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, _: &Matrix) -> AngularInertia {
        self.clone()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &Scalar, pt: &Vect) -> AngularInertia {
        *self + Mat1::new(mass * na::sqnorm(pt))
    }
}

#[dim3]
impl InertiaTensor for AngularInertia {
    #[inline]
    fn apply(&self, av: &Orientation) -> Orientation {
        *self * *av
    }

    #[inline]
    fn to_world_space(&self, t: &Matrix) -> AngularInertia {
        let inv = na::inv(&t.rotation).unwrap();
        *t.rotation.submat() * *self * *inv.submat()
    }

    #[inline]
    fn to_relative_wrt_point(&self, mass: &Scalar, pt: &Vect) -> AngularInertia {
        let diag  = na::sqnorm(pt);
        let diagm = Mat3::new(
            diag.clone(), na::zero(),   na::zero(),
            na::zero(),   diag.clone(), na::zero(),
            na::zero(),   na::zero(),   diag
        );

        *self + (diagm - na::outer(pt, pt)) * *mass
    }
}

#[dim4]
impl InertiaTensor for AngularInertia {
    #[inline]
    fn apply(&self, _: &Orientation) -> Orientation {
        fail!("Not yet implemented.")
    }

    #[inline]
    fn to_world_space(&self, _: &Matrix) -> AngularInertia {
        fail!("Not yet implemented.")
    }

    #[inline]
    fn to_relative_wrt_point(&self, _: &Scalar, _: &Vect) -> AngularInertia {
        fail!("Not yet implemented.")
    }
}
