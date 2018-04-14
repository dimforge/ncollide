use math::{Isometry, Point};
use shape::{FeatureId, Shape};
use query::{PointProjection, PointQuery};

impl<P, M> PointQuery<P, M> for Shape<N>
where
    N: Real,
    M: Isometry<P>,
{
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> PointProjection<P> {
        self.as_point_query()
            .expect("No PointQuery implementation for the underlying shape.")
            .project_point(m, pt, solid)
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &P) -> (PointProjection<P>, FeatureId) {
        self.as_point_query()
            .expect("No PointQuery implementation for the underlying shape.")
            .project_point_with_feature(m, pt)
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> N {
        self.as_point_query()
            .expect("No PointQuery implementation for the underlying shape.")
            .distance_to_point(m, pt, solid)
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &P) -> bool {
        self.as_point_query()
            .expect("No PointQuery implementation for the underlying shape.")
            .contains_point(m, pt)
    }
}
