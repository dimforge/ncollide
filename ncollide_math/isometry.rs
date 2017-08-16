use std::fmt::Display;
use std::ops::Mul;
use alga::general::{Id, Real};
use alga::linear::{Isometry as AlgaIsometry};
use na::{self, DefaultAllocator, Translation, Rotation, UnitQuaternion, MatrixN,
         UnitComplex, Matrix2, Matrix3};
use na::dimension::DimName;
use na::storage::Owned;
use na::allocator::Allocator;
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

impl<N, D, P: Point> Isometry<P> for Translation<N, D>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          Owned<N, D>: Copy + Sync + Send + 'static,
          DefaultAllocator: Allocator<N, D> + Allocator<usize, D>,
          Translation<N, D>: AlgaIsometry<P> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        *v
    }
}

impl<N, D, P: Point> Isometry<P> for Rotation<N, D>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          Owned<N, D, D>: Copy + Sync + Send + 'static,
          DefaultAllocator: Allocator<N, D, D> +
                            Allocator<usize, D, D>,
          Rotation<N, D>: AlgaIsometry<P>,
          MatrixN<N, D>: Mul<P::Vector, Output = P::Vector> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        self.matrix().abs() * *v
    }
}

impl<N, P: Point> Isometry<P> for UnitQuaternion<N>
    where N: Real,
          UnitQuaternion<N>: AlgaIsometry<P>,
          Matrix3<N>: Mul<P::Vector, Output = P::Vector> {
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

impl<N, D, R, P: Point> Isometry<P> for na::Isometry<N, D, R>
    where N: Real,
          D: Send + Sync + 'static + DimName,
          Owned<N, D>: Copy + Sync + Send + 'static,
          DefaultAllocator: Allocator<N, D, D> + Allocator<N, D> + Allocator<usize, D>,
          na::Isometry<N, D, R>: AlgaIsometry<P>,
          R: Isometry<P> {
    #[inline]
    fn absolute_rotate_vector(&self, v: &P::Vector) -> P::Vector {
        self.rotation.absolute_rotate_vector(v)
    }
}
