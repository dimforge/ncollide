use bounding_volume::AABB;
use math::Isometry;
use na::Real;
use partitioning::BVT;
use shape::Shape;

/// Trait implemented by shapes composed of multiple simpler shapes.
///
/// A composite shape is composed of several shapes. Typically, it is a convex decomposition of
/// a concave shape.
pub trait CompositeShape<N: Real> {
    /// The number of sub-shape in this composide sahpe.
    fn nparts(&self) -> usize;
    /// Applies a function to each sub-shape of this concave shape.
    fn map_part_at(&self, usize, &mut FnMut(usize, &Isometry<N>, &Shape<N>));
    /// Applies a transformation matrix and a function to each sub-shape of this concave
    /// shape.
    fn map_transformed_part_at(&self, usize, m: &Isometry<N>, &mut FnMut(usize, &Isometry<N>, &Shape<N>));

    // FIXME: the following two methods really are not generic enough.
    /// Gets the AABB of the shape identified by the index `i`.
    fn aabb_at(&self, i: usize) -> AABB<N>;
    /// Gets the acceleration structure of the concave shape.
    fn bvt(&self) -> &BVT<usize, AABB<N>>;
}
