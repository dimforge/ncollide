use na::Real;

/// The type of elements used to describe a deformation on a collision object.
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

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N], indices: &[DeformationIndex]);
}