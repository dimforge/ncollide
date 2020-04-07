use crate::math::{Isometry, Point};
use crate::query::{PointProjection, PointQuery, PointQueryWithLocation};
use crate::shape::{FeatureId, HeightField, TrianglePointLocation};
use na::{self, RealField};

impl<N: RealField> PointQuery<N> for HeightField<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &Point<N>, _: bool) -> PointProjection<N> {
        let mut smallest_dist = N::max_value();
        let mut best_proj = PointProjection::new(false, *point);

        #[cfg(feature = "dim2")]
        let iter = self.segments();
        #[cfg(feature = "dim3")]
        let iter = self.triangles();
        for elt in iter {
            let proj = elt.project_point(m, point, false);
            let dist = na::distance_squared(point, &proj.point);

            if dist < smallest_dist {
                smallest_dist = dist;
                best_proj = proj;
            }
        }

        best_proj
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        point: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        // FIXME: compute the feature properly.
        (self.project_point(m, point, false), FeatureId::Unknown)
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, _m: &Isometry<N>, _point: &Point<N>) -> bool {
        false
    }
}

impl<N: RealField> PointQueryWithLocation<N> for HeightField<N> {
    type Location = (usize, TrianglePointLocation<N>);

    #[inline]
    fn project_point_with_location(
        &self,
        _m: &Isometry<N>,
        _point: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location) {
        unimplemented!()
    }
}
