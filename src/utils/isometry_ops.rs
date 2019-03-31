use crate::math::Vector;
#[cfg(feature = "dim2")]
use na::Isometry2;
#[cfg(feature = "dim3")]
use na::Isometry3;
use na::{RealField, Unit};

/// Extra operations with isometries.
pub trait IsometryOps<N: RealField> {
    /// Transform a vector by the absolute value of the homogeneous matrix
    /// equivalent to `self`.
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N>;
    /// Transform a unit vector by the inverse of `self`.
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>>;
}

#[cfg(feature = "dim2")]
impl<N: RealField> IsometryOps<N> for Isometry2<N> {
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().into_inner().abs() * *v
    }

    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        let v = self.inverse_transform_vector(v.as_ref());
        Unit::new_unchecked(v)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> IsometryOps<N> for Isometry3<N> {
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().into_inner().abs() * *v
    }

    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        let v = self.inverse_transform_vector(v.as_ref());
        Unit::new_unchecked(v)
    }
}
