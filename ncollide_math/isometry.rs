use std::fmt::{Debug, Display};
use std::ops::Mul;
use alga::general::{Id, Real};
use alga::linear::Isometry as AlgaIsometry;
use na::{self, DefaultAllocator, Matrix2, Matrix3, MatrixN, Rotation, Translation, Unit,
         UnitComplex, UnitQuaternion};
use na::dimension::DimName;
use na::storage::Owned;
use na::allocator::Allocator;
use Point;

/// Trait implemented by isometry types usable by ncollide.
pub trait Isometry<N: Real>: Send + Sync + 'static + Display + AlgaIsometry<P> {
    /// Computes the product `abs(rot) * v` where `abs(self)` is the absolute value of the matrix
    /// representation of `self.rotation()`.
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N>;

    /// Applies the isometry to a unit vector.
    fn transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        Unit::new_unchecked(self.transform_vector(v.as_ref()))
    }
    
    /// Applies the inverse isometry to a unit vector.
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        Unit::new_unchecked(self.inverse_transform_vector(v.as_ref()))
    }
}

impl<N: Real> Isometry<P> for Id {
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        *v
    }
}

impl<N, D, N: Real> Isometry<P> for Translation<N, D>
where
    N: Real,
    D: Send + Sync + 'static + DimName,
    Owned<N, D>: Copy + Sync + Send + 'static,
    DefaultAllocator: Allocator<N, D> + Allocator<usize, D>,
    Translation<N, D>: AlgaIsometry<P>,
{
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        *v
    }
}

impl<N, D, N: Real> Isometry<P> for Rotation<N, D>
where
    N: Real,
    D: Send + Sync + 'static + DimName,
    Owned<N, D, D>: Copy + Sync + Send + 'static,
    DefaultAllocator: Allocator<N, D, D> + Allocator<usize, D, D>,
    Rotation<N, D>: AlgaIsometry<P>,
    MatrixN<N, D>: Mul<Vector<N>, Output = Vector<N>>,
{
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.matrix().abs() * *v
    }
}

impl<N, N: Real> Isometry<P> for UnitQuaternion<N>
where
    N: Real,
    UnitQuaternion<N>: AlgaIsometry<P>,
    Matrix3<N>: Mul<Vector<N>, Output = Vector<N>>,
{
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.to_rotation_matrix().unwrap().abs() * *v
    }
}

impl<N, N: Real> Isometry<P> for UnitComplex<N>
where
    N: Real,
    UnitComplex<N>: AlgaIsometry<P>,
    Matrix2<N>: Mul<Vector<N>, Output = Vector<N>>,
{
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        let r = self.complex().re.abs();
        let i = self.complex().im.abs();
        Matrix2::new(r, i, i, r) * *v
    }
}

impl<N, D, R, N: Real> Isometry<P> for na::Isometry<N, D, R>
where
    N: Real,
    D: Send + Sync + 'static + DimName,
    Owned<N, D>: Copy + Sync + Send + 'static,
    DefaultAllocator: Allocator<N, D, D> + Allocator<N, D> + Allocator<usize, D>,
    na::Isometry<N, D, R>: AlgaIsometry<P>,
    R: Isometry<P>,
{
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.absolute_transform_vector(v)
    }
}
