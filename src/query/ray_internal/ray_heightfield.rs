use bounding_volume::AABB;
use math::Isometry;
use na::{Point2, Real, Vector3};
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use query::{ray_internal, Ray, RayCast, RayIntersection};
use shape::{CompositeShape, HeightField, FeatureId};

impl<N: Real> RayCast<N> for HeightField<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        None
//        unimplemented!()
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>>
    {
        None
//        unimplemented!()
    }
}