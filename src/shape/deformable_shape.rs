use crate::query::LocalShapeApproximation;
use na::RealField;

/// The type of elements used to describe a deformation on a collision object.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum DeformationsType {
    /// Deformations described as scalars.
    Scalars,
    /// Deformations described as vectors.
    Vectors,
    /// Deformations described as isometries.
    Isometries,
}

/// Trait implemented by deformable shapes.
pub trait DeformableShape<N: RealField> {
    /// The type of degrees of freedom this shape has.
    fn deformations_type(&self) -> DeformationsType;

    //    /// Applies a deformation to all the degrees of freedom of this shape.
    //    fn deform_all(&mut self, coords: &[N], indices: &[usize]);

    /// Updates some the degrees of freedom of this shape.
    //    fn set_partial_deformations(&mut self, coords: &[N], indices: &[DeformationIndex]);

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N]);

    /// Updates the given local approximation of this shape.
    fn update_local_approximation(&self, coords: &[N], approx: &mut LocalShapeApproximation<N>);
}
