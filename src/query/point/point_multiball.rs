use crate::math::{Isometry, Point, Vector, Translation};
use na::{self, RealField};
use crate::query::{PointProjection, PointQuery, PointQueryWithLocation};
use crate::shape::{FeatureId, Multiball, Ball, TrianglePointLocation};


impl<N: RealField> PointQuery<N> for Multiball<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        self.project_point_with_feature(m, pt).0
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        let ball = Ball::new(self.radius());
        let mut smallest_sqdist = N::max_value();
        let mut result = (PointProjection::new(false, *pt), FeatureId::Unknown);

        // FIXME: use the HGrid structure to avoid the following brute-force approach.
        for (i, c) in self.centers().iter().enumerate() {
            let ball_m = m * Translation::from(c.coords);
            let proj = ball.project_point(&ball_m, pt, true);

            if proj.is_inside {
                return (proj, FeatureId::Face(i));
            }

            let candidate_sqdist = na::distance_squared(pt, &proj.point);
            if candidate_sqdist < smallest_sqdist {
                result = (proj, FeatureId::Face(i));
                smallest_sqdist = candidate_sqdist;
            }
        }

        result
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}
