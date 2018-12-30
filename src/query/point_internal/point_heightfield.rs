use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point};
use na::{self, Real};
use crate::query::{
    PointProjection, PointQuery, PointQueryWithLocation,
};
use crate::shape::{CompositeShape, FeatureId, HeightField, TrianglePointLocation};
use crate::utils::IsometryOps;

impl<N: Real> PointQuery<N> for HeightField<N> {
    #[inline]
    fn project_point(&self, _m: &Isometry<N>, _point: &Point<N>, _solid: bool) -> PointProjection<N> {
        unimplemented!()
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        _: &Isometry<N>,
        _: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, _m: &Isometry<N>, _point: &Point<N>) -> bool {
        false
//        unimplemented!()

    }
}

impl<N: Real> PointQueryWithLocation<N> for HeightField<N> {
    type Location = (usize, TrianglePointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        _m: &Isometry<N>,
        _point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location)
    {
        unimplemented!()
    }
}
