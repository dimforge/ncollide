use bounding_volume::AABB;
use math::{Isometry, Point};
use na::{self, Real};
use query::{
    PointProjection, PointQuery, PointQueryWithLocation,
};
use shape::{CompositeShape, FeatureId, HeightField, TrianglePointLocation};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for HeightField<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, solid: bool) -> PointProjection<N> {
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
    fn contains_point(&self, m: &Isometry<N>, point: &Point<N>) -> bool {
        false
//        unimplemented!()

    }
}

impl<N: Real> PointQueryWithLocation<N> for HeightField<N> {
    type Location = (usize, TrianglePointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location)
    {
        unimplemented!()
    }
}
