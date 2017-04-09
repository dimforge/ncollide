use na;
use shape::Segment;
use query::{PointQuery, PointProjection, RichPointQuery};
use math::{Point, Isometry};


impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Segment<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        self.project_point_with_extra_info(m, pt, solid).0
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
        let bcoords;

        if ab_ap <= na::zero() {
            // Voronoï region of vertex 'a'.
            bcoords = na::zero();
            proj    = m.transform_point(self.a());
        }
        else if ab_ap >= sqnab {
            // Voronoï region of vertex 'b'.
            bcoords = na::one();
            proj    = m.transform_point(self.b());
        }
        else {
            assert!(sqnab != na::zero());

            // Voronoï region of the segment interior.
            bcoords = ab_ap / sqnab;
            proj    = m.transform_point(&(*self.a() + ab * bcoords));
        }

        // FIXME: is this acceptable?
        let inside = relative_eq!(proj, *pt);

        (PointProjection::new(inside, proj), bcoords)
    }
}
