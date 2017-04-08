use na;
use shape::Segment;
use query::{PointQuery, PointProjection, RichPointQuery};
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Segment<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let (projection, _) = self.project_point_with_extra_info(m, pt, solid);
        projection
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}

impl<P: Point, M: Isometry<P>> RichPointQuery<P, M> for Segment<P> {
    type ExtraInfo = P::Real;

    #[inline]
    fn project_point_with_extra_info(&self, m: &M, pt: &P, _: bool)
        -> (PointProjection<P>, Self::ExtraInfo)
    {
        let ls_pt = m.inverse_transform_point(pt);
        let ab    = *self.b() - *self.a();
        let ap    = ls_pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::norm_squared(&ab);

        let proj;
        let position_on_segment;

        if ab_ap <= na::zero() {
            // Voronoï region of vertex 'a'.
            position_on_segment = na::zero();
            proj = m.transform_point(self.a());
        }
        else if ab_ap >= sqnab {
            // Voronoï region of vertex 'b'.
            position_on_segment = na::one();
            proj = m.transform_point(self.b());
        }
        else {
            assert!(sqnab != na::zero());

            // Voronoï region of the segment interior.
            position_on_segment = ab_ap / sqnab;
            proj = m.transform_point(&(*self.a() + ab * position_on_segment));
        }

        // FIXME: is this acceptable?
        let inside = relative_eq!(proj, *pt);

        (PointProjection::new(inside, proj), position_on_segment)
    }
}
