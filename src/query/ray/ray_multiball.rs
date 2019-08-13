use crate::math::{Isometry, Point, Vector, Translation};
use na::{self, RealField};
use crate::query::{RayCast, Ray, RayIntersection};
use crate::shape::{FeatureId, Multiball, Ball, TrianglePointLocation};


impl<N: RealField> RayCast<N> for Multiball<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        // FIXME
        None
    }
}
