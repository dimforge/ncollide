use alga::linear::ProjectiveTransformation;
use na::{Real, Unit};
use math::{Vector, Point};
#[cfg(feature = "dim2")]
use na::Isometry2;
#[cfg(feature = "dim3")]
use na::Isometry3;

/// Extra operations with isometries.
pub trait IsometryOps<N: Real> {
    /// Transform a vector by the absolute value of the homogeneous matrix
    /// equivalent to `self`.
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N>;
    /// Transform a vector by the inverse of `self`.
    fn inverse_transform_vector(&self, v: &Vector<N>) -> Vector<N>;
    /// Transform a point by the inverse of `self`.
    fn inverse_transform_point(&self, p: &Point<N>) -> Point<N>;
    /// Transform a unit vector by the inverse of `self`.
    fn inverse_transform_unit_vector(&self, v: &Unit<Vector<N>>) -> Unit<Vector<N>> {
        let v = self.inverse_transform_vector(v.as_ref());
        Unit::new_unchecked(v)
    }
}

#[cfg(feature = "dim2")]
impl<N: Real> IsometryOps<N> for Isometry2<N> {
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().unwrap().abs() * *v
    }
    fn inverse_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        ProjectiveTransformation::inverse_transform_vector(self, v)
    }
    fn inverse_transform_point(&self, p: &Point<N>) -> Point<N> {
        ProjectiveTransformation::inverse_transform_point(self, p)
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> IsometryOps<N> for Isometry3<N> {
    fn absolute_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        self.rotation.to_rotation_matrix().unwrap().abs() * *v
    }
    fn inverse_transform_vector(&self, v: &Vector<N>) -> Vector<N> {
        ProjectiveTransformation::inverse_transform_vector(self, v)
    }
    fn inverse_transform_point(&self, p: &Point<N>) -> Point<N> {
        ProjectiveTransformation::inverse_transform_point(self, p)
    }
}