use na::Real;
use crate::query::LocalShapeApproximation;

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

/// Mapping between source and target degrees of freedom for copying deformations.
pub struct DeformationIndex {
    /// Index on the source buffer of one deformation of type DeformationType to be copied.
    pub source: usize,
    /// Index on the target buffer of one deformation of type DeformationType to be copied.
    pub target: usize,
}

/// Trait implemented by deformable shapes.
pub trait DeformableShape<N: Real> {
    /// The type of degrees of freedom this shape has.
    fn deformations_type(&self) -> DeformationsType;

    //    /// Applies a deformation to all the degrees of freedom of this shape.
    //    fn deform_all(&mut self, coords: &[N], indices: &[usize]);

    /// Updates some the degrees of freedom of this shape.
    //    fn set_partial_deformations(&mut self, coords: &[N], indices: &[DeformationIndex]);

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N], indices: Option<&[usize]>);

    /// Updates the given local approximation of this shape.
    fn update_local_approximation(
        &self,
        coords: &[N],
        indices: Option<&[usize]>,
        approx: &mut LocalShapeApproximation<N>,
    );
}
