use crate::math::Vector;
#[cfg(feature = "dim2")]
use na::Isometry2;
#[cfg(feature = "dim3")]
use na::Isometry3;
use na::{RealField, Unit};

/// Extra operations with isometries.
pub trait IsometryOps<N: RealField + Copy> {
    /// Transform a vector by the absolute value of the homogeneous matrix
    /// equivalent to `self`.
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N>;
    /// Transform a unit vector by the inverse of `self`.
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>>;
    /// Interpolates between two isometries, using LERP for the translation part and SLERP for the rotation part.
    fn lerp_slerp(&self, other: &Self, t: N) -> Self;
}

#[cfg(feature = "dim2")]
impl<N: RealField + Copy> IsometryOps<N> for Isometry2<N> {
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().into_inner().abs() * *v
    }

    #[inline]
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        let v = self.inverse_transform_vector(v.as_ref());
        Unit::new_unchecked(v)
    }

    #[inline]
    fn lerp_slerp(&self, other: &Self, t: N) -> Self {
        let tr = self.translation.vector.lerp(&other.translation.vector, t);
        let ang = self.rotation.angle() * (N::one() - t) + other.rotation.angle() * t;
        Self::new(tr, ang)
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField + Copy> IsometryOps<N> for Isometry3<N> {
    #[inline]
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().into_inner().abs() * *v
    }

    #[inline]
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        let v = self.inverse_transform_vector(v.as_ref());
        Unit::new_unchecked(v)
    }

    #[inline]
    fn lerp_slerp(&self, other: &Self, t: N) -> Self {
        let tr = self.translation.vector.lerp(&other.translation.vector, t);
        let rot = self.rotation.slerp(&other.rotation, t);
        Self::from_parts(tr.into(), rot)
    }
}
