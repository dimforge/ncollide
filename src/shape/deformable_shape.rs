use na::Real;

/// The type of elements used to describe a deformation on a collision object.
pub enum DeformationType {
    /// Deformations described as scalars.
    Scalar,
    /// Deformations described as vectors.
    Vectors,
    /// Deformations described as isometries.
    Isometries,
}

/// Trait implemented by deformable shapes.
pub trait DeformableShape<N: Real> {
    /// The type of degrees of freedom this shape has.
    fn deformation_type(&self) -> DeformationType;

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N], indices: &[usize]);
}