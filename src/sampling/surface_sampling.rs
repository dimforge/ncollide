use na::RealField;
use crate::shape::FeatureId;
use crate::math::Point;

/// Trait implemented by shapes that are able to output a point-based sample of their surface.
pub trait SurfaceSampling<N: RealField> {
    /// Sample a feature with points spaced by a distance of at most `spacing`.
    fn sample_feature(&self, feature: FeatureId, spacing: N, out: &mut Vec<Point<N>>);
    /// Sample the whole shape with points spaced a distance of by at most `spacing`.
    fn sample_surface(&self, spacing: N, out: &mut Vec<Point<N>>);
}