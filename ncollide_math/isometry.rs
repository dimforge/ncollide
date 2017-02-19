use std::fmt::Display;
use std::ops::Mul;
use alga::general::{Id, Real};
use alga::linear::{Isometry as AlgaIsometry};
use na::{TranslationBase, RotationBase, UnitQuaternionBase, IsometryBase, Matrix, OwnedMatrix,
         UnitComplex, Matrix2};
use na::dimension::{DimName, U1, U3, U4};
use na::storage::OwnedStorage;
use na::allocator::{Allocator, OwnedAllocator};
use ::Point;


/// Trait implemented by isometry types usable by ncollide.
pub trait Isometry<P: Point>: Send + Sync + 'static + Display + AlgaIsometry<P> {
    /// Computes the product `abs(rot) * v` where `abs(self)` is the absolute value of the matrix
    /// representation of `self.rotation()`.
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector;
}

impl<P: Point> Isometry<P> for Id {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        *v
    }
}

impl<N, D, S, P: Point> Isometry<P> for TranslationBase<N, D, S>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          S: Send + Sync + 'static + OwnedStorage<N, D, U1>,
          S::Alloc: OwnedAllocator<N, D, U1, S> +
                    Allocator<usize, D, U1>,
          TranslationBase<N, D, S>: AlgaIsometry<P> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        *v
    }
}

impl<N, D, S, P: Point> Isometry<P> for RotationBase<N, D, S>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          S: Send + Sync + 'static + OwnedStorage<N, D, D>,
          S::Alloc: OwnedAllocator<N, D, D, S> +
                    Allocator<usize, D, D>,
          RotationBase<N, D, S>: AlgaIsometry<P>,
          Matrix<N, D, D, S>: Mul<P::Vector, Output = P::Vector> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        self.matrix().abs() * *v
    }
}

impl<N, S, P: Point> Isometry<P> for UnitQuaternionBase<N, S>
    where N: Real,
          S: Send + Sync + 'static + OwnedStorage<N, U4, U1>,
          S::Alloc: OwnedAllocator<N, U4, U1, S> +
                    Allocator<N, U3, U1> +
                    Allocator<N, U3, U3> +
                    Allocator<usize, U4, U1>,
          UnitQuaternionBase<N, S>: AlgaIsometry<P>,
          OwnedMatrix<N, U3, U3, S::Alloc>: Mul<P::Vector, Output = P::Vector> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        self.to_rotation_matrix().unwrap().abs() * *v
    }
}

impl<N, P: Point> Isometry<P> for UnitComplex<N>
    where N: Real,
          UnitComplex<N>: AlgaIsometry<P>,
          Matrix2<N>: Mul<P::Vector, Output = P::Vector> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        let r = self.complex().re.abs();
        let i = self.complex().im.abs();
        Matrix2::new(r, i, i, r) * *v
    }
}

impl<N, D, S, R, P: Point> Isometry<P> for IsometryBase<N, D, S, R>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          S: Send + Sync + 'static + OwnedStorage<N, D, U1>,
          S::Alloc: OwnedAllocator<N, D, U1, S> +
                    Allocator<usize, D, U1>,
          IsometryBase<N, D, S, R>: AlgaIsometry<P>,
          R: Isometry<P> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        self.rotation.absolute_rotate_vector(v)
    }
}
