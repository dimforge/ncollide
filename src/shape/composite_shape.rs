use crate::bounding_volume::AABB;
use crate::math::Isometry;
use na::Real;
use crate::partitioning::BVHImpl;
use crate::shape::{FeatureId, Shape};
use crate::query::{ContactPreprocessor, ContactPrediction};

/// Trait implemented by shapes composed of multiple simpler shapes.
///
/// A composite shape is composed of several shapes. Typically, it is a convex decomposition of
/// a concave shape.
pub trait CompositeShape<N: Real> {
    /// The number of sub-shape in this composide shape.
    fn nparts(&self) -> usize;

    /// Applies a transformation matrix and a function to each sub-shape of this concave
    /// shape.
    fn map_part_at(
        &self,
        _: usize,
        m: &Isometry<N>,
        _: &mut FnMut(&Isometry<N>, &Shape<N>),
    );

    /// Applies a transformation matrix and a function to each sub-shape of this concave
    /// shape.
    fn map_part_and_preprocessor_at(
        &self,
        _: usize,
        m: &Isometry<N>,
        prediction: &ContactPrediction<N>,
        _: &mut FnMut(&Isometry<N>, &Shape<N>, &ContactPreprocessor<N>),
    );


    // FIXME: the following two methods are not generic enough.
    /// Gets the AABB of the shape identified by the index `i`.
    fn aabb_at(&self, i: usize) -> AABB<N>;

    /// Gets the acceleration structure of the concave shape.
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>>;
}
